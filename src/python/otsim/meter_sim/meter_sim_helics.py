from __future__ import annotations

"""
Meter Simulator with HELICS integration - OT-sim module.

Receives values from HELICS (via the IO module), computes aggregate load for
multiple EVs from three-phase currents, and exposes values as Modbus TCP
holding registers.

Modbus holding registers (float32, big-endian, two regs each):
  HR 0-1 : energy-consumption (kWh, passthrough)
  HR 2-3 : max-demand         (kW, max observed)
  HR 4-5 : total-watts        (W, aggregate across EVs)
  HR 6-7 : one-minute-usage   (Wh, rolling 1-minute usage)

XML configuration example::

  <meter-sim-helics name="meter-1">
    <listen-address>0.0.0.0</listen-address>
    <port>5020</port>
    <unit-id>1</unit-id>
    <tags>
      <energy-consumption>meter-1.energy-consumption</energy-consumption>
      <max-demand>meter-1.max-demand</max-demand>
      <current-l1-suffix>.current-l1</current-l1-suffix>
      <current-l2-suffix>.current-l2</current-l2-suffix>
      <current-l3-suffix>.current-l3</current-l3-suffix>
    </tags>
    <line-voltage>230.0</line-voltage>
    <power-factor>1.0</power-factor>
  </meter-sim-helics>

Current tags are grouped per EV by the prefix before the suffix. Example:

  evse-1.current-l1, evse-1.current-l2, evse-1.current-l3
  evse-2.current-l1, evse-2.current-l2, evse-2.current-l3

The module sums each EV's three-phase power and stores the fleet total.

The IO module must map HELICS keys to these tags:

  <subscription>
    <key>OpenDSS/meter-1.energy-consumption</key>
    <type>double</type>
    <tag>meter-1.energy-consumption</tag>
  </subscription>
  <subscription>
    <key>OpenDSS/meter-1.max-demand</key>
    <type>double</type>
    <tag>meter-1.max-demand</tag>
  </subscription>
"""

import signal, struct, sys, threading, time, typing
from collections import deque

import otsim.msgbus.envelope as envelope
import xml.etree.ElementTree as ET

from otsim.msgbus.envelope   import Envelope, Point
from otsim.msgbus.pusher     import Pusher
from otsim.msgbus.subscriber import Subscriber

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusDeviceContext,
    ModbusServerContext,
)
from pymodbus.server import StartTcpServer


DEFAULT_PUB_ENDPOINT = 'tcp://127.0.0.1:5678'
DEFAULT_PULL_ENDPOINT = 'tcp://127.0.0.1:1234'


def _float_to_registers(value: float) -> typing.List[int]:
    packed    = struct.pack('>f', float(value))
    high, low = struct.unpack('>HH', packed)
    return [high, low]


def _read_endpoint(el: ET.Element, tag: str, default: str) -> str:
    value = el.findtext(tag, default=default)
    if value is None:
      return default

    stripped = value.strip()
    return stripped if stripped else default


class MeterSimHelics:
  """Receives energy-consumption and max-demand from HELICS via the IO module
  and exposes them as Modbus TCP holding registers."""

  def __init__(self: 'MeterSimHelics', pub: str, pull: str, el: ET.Element):
    self.running = False

    self.name    = el.get('name', default='ot-sim-meter-sim-helics')
    self.address = el.findtext('listen-address', default='0.0.0.0')
    self.port    = int(el.findtext('port',        default='5020'))
    self.unit_id = int(el.findtext('unit-id',     default='1'))

    tags = el.find('tags') or ET.Element('tags')

    self.tag_energy_consumption = tags.findtext(
      'energy-consumption', default='meter.energy-consumption'
    )
    self.tag_max_demand       = tags.findtext('max-demand',        default='meter.max-demand')
    self.tag_total_watts      = tags.findtext('total-watts',       default=f'{self.name}.total-watts')
    self.tag_one_minute_usage = tags.findtext('one-minute-usage',  default=f'{self.name}.one-minute-usage')
    self.current_l1_suffix    = tags.findtext('current-l1-suffix', default='.current-l1')
    self.current_l2_suffix    = tags.findtext('current-l2-suffix', default='.current-l2')
    self.current_l3_suffix    = tags.findtext('current-l3-suffix', default='.current-l3')

    self.line_voltage = float(el.findtext('line-voltage', default='230.0'))
    self.power_factor = float(el.findtext('power-factor', default='1.0'))

    self._lock               = threading.Lock()
    self._energy_consumption = 0.0
    self._max_demand         = 0.0
    self._total_watts        = 0.0
    self._one_minute_usage_wh = 0.0
    self._ev_phase_currents: typing.Dict[str, typing.Dict[str, float]] = {}
    self._watts_history: typing.Deque[typing.Tuple[float, float]] = deque()

    block        = ModbusSequentialDataBlock(0, [0] * 8)
    store        = ModbusDeviceContext(hr=block)
    self.context = ModbusServerContext(devices={self.unit_id: store}, single=False)

    pub_endpoint  = _read_endpoint(el, 'pub-endpoint',  pub)
    pull_endpoint = _read_endpoint(el, 'pull-endpoint', pull)
    self.subscriber = Subscriber(pub_endpoint)
    self.subscriber.add_status_handler(self.handle_status)
    self.pusher = Pusher(pull_endpoint)


  def start(self: 'MeterSimHelics'):
    self.running = True
    self.subscriber.start('RUNTIME')
    threading.Thread(target=self._serve, daemon=True).start()


  def stop(self: 'MeterSimHelics'):
    self.running = False
    self.subscriber.stop()


  def handle_status(self: 'MeterSimHelics', env: Envelope):
    status = envelope.status_from_envelope(env)

    if not status:
      return

    updated = False

    for point in status.get('measurements', []):
      tag = point['tag']
      val = float(point['value'])

      if tag == self.tag_energy_consumption:
        with self._lock:
          self._energy_consumption = val
        updated = True
      elif tag == self.tag_max_demand:
        with self._lock:
          self._max_demand = val
        updated = True
      elif self._try_update_phase_current(tag, val):
        updated = True

    if updated:
      self._recompute_totals()
      self._write_registers()
      self._publish_computed()


  def _try_update_phase_current(self: 'MeterSimHelics', tag: str, val: float) -> bool:
    phase = None
    suffix = ''

    if tag.endswith(self.current_l1_suffix):
      phase = 'l1'
      suffix = self.current_l1_suffix
    elif tag.endswith(self.current_l2_suffix):
      phase = 'l2'
      suffix = self.current_l2_suffix
    elif tag.endswith(self.current_l3_suffix):
      phase = 'l3'
      suffix = self.current_l3_suffix

    if phase is None:
      return False

    vehicle = tag[:-len(suffix)] if suffix else tag
    if not vehicle:
      return False

    with self._lock:
      if vehicle not in self._ev_phase_currents:
        self._ev_phase_currents[vehicle] = {'l1': 0.0, 'l2': 0.0, 'l3': 0.0}
      self._ev_phase_currents[vehicle][phase] = max(0.0, val)

    return True


  def _recompute_totals(self: 'MeterSimHelics'):
    now = time.monotonic()

    with self._lock:
      total_watts = 0.0

      for phases in self._ev_phase_currents.values():
        total_watts += (
          self.line_voltage * phases['l1'] +
          self.line_voltage * phases['l2'] +
          self.line_voltage * phases['l3']
        ) * self.power_factor

      self._total_watts = total_watts
      self._max_demand = max(self._max_demand, total_watts / 1000.0)

      self._watts_history.append((now, total_watts))
      cutoff = now - 60.0
      while self._watts_history and self._watts_history[0][0] < cutoff:
        self._watts_history.popleft()

      if self._watts_history:
        avg_watts = sum(sample for _, sample in self._watts_history) / len(self._watts_history)
        self._one_minute_usage_wh = avg_watts / 60.0
      else:
        self._one_minute_usage_wh = 0.0


  def _publish_computed(self: 'MeterSimHelics'):
    with self._lock:
      tw        = self._total_watts
      one_min   = self._one_minute_usage_wh

    points: typing.List[Point] = [
      {'tag': self.tag_total_watts,      'value': tw,      'ts': 0},
      {'tag': self.tag_one_minute_usage, 'value': one_min, 'ts': 0},
    ]
    env = envelope.new_status_envelope(self.name, {'measurements': points})
    self.pusher.push('RUNTIME', env)


  def _write_registers(self: 'MeterSimHelics'):
    with self._lock:
      ec = self._energy_consumption
      md = self._max_demand
      tw = self._total_watts
      one_min_wh = self._one_minute_usage_wh

    registers = (
      _float_to_registers(ec) +
      _float_to_registers(md) +
      _float_to_registers(tw) +
      _float_to_registers(one_min_wh)
    )
    self.context[self.unit_id].setValues(3, 0, registers)


  def _serve(self: 'MeterSimHelics'):
    StartTcpServer(context=self.context, address=(self.address, self.port))


def main():
  if len(sys.argv) < 2:
    print('no config file provided')
    sys.exit(1)

  tree = ET.parse(sys.argv[1])

  root = tree.getroot()
  assert root.tag == 'ot-sim'

  mb = root.find('message-bus')

  if mb:
    pub  = _read_endpoint(mb, 'pub-endpoint',  DEFAULT_PUB_ENDPOINT)
    pull = _read_endpoint(mb, 'pull-endpoint', DEFAULT_PULL_ENDPOINT)
  else:
    pub  = DEFAULT_PUB_ENDPOINT
    pull = DEFAULT_PULL_ENDPOINT

  modules: typing.List[MeterSimHelics] = []

  for ms in root.findall('meter-sim-helics'):
    module = MeterSimHelics(pub, pull, ms)
    module.start()
    modules.append(module)

  waiter = threading.Event()

  def handler(*_):
    waiter.set()

  signal.signal(signal.SIGINT, handler)
  waiter.wait()

  for module in modules:
    module.stop()
