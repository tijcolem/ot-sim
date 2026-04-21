from __future__ import annotations

"""
Meter Simulator with HELICS integration - OT-sim module

Receives Power Consumption (kWh) and Maximum Demand (kW) from a HELICS
co-simulation via the IO module and exposes them as Modbus TCP holding
registers.

Modbus holding registers (float32, big-endian, two regs each):
  HR 0–1 : energy-consumption (kWh, from HELICS)
  HR 2–3 : max-demand         (kW,  from HELICS)

XML configuration example::

  <meter-sim-helics name="meter-1">
    <listen-address>0.0.0.0</listen-address>
    <port>5020</port>
    <unit-id>1</unit-id>
    <tags>
      <energy-consumption>meter-1.energy-consumption</energy-consumption>
      <max-demand>meter-1.max-demand</max-demand>
    </tags>
  </meter-sim-helics>

The IO module must be configured with subscriptions that map the HELICS keys
to the tags named in <energy-consumption> and <max-demand>:

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

import signal, struct, sys, threading, typing

import otsim.msgbus.envelope as envelope
import xml.etree.ElementTree as ET

from otsim.msgbus.envelope   import Envelope
from otsim.msgbus.subscriber import Subscriber

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusDeviceContext,
    ModbusServerContext,
)
from pymodbus.server import StartTcpServer


def _float_to_registers(value: float) -> typing.List[int]:
    packed    = struct.pack('>f', float(value))
    high, low = struct.unpack('>HH', packed)
    return [high, low]


class MeterSimHelics:
  """Receives energy-consumption and max-demand from HELICS via the IO module
  and exposes them as Modbus TCP holding registers HR 0–3."""

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
    self.tag_max_demand = tags.findtext('max-demand', default='meter.max-demand')

    self._lock               = threading.Lock()
    self._energy_consumption = 0.0
    self._max_demand         = 0.0

    block        = ModbusSequentialDataBlock(0, [0] * 4)
    store        = ModbusDeviceContext(hr=block)
    self.context = ModbusServerContext(devices={self.unit_id: store}, single=False)

    pub_endpoint = el.findtext('pub-endpoint', default=pub)
    self.subscriber = Subscriber(pub_endpoint)
    self.subscriber.add_status_handler(self.handle_status)


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

    if updated:
      self._write_registers()


  def _write_registers(self: 'MeterSimHelics'):
    with self._lock:
      ec = self._energy_consumption
      md = self._max_demand

    registers = _float_to_registers(ec) + _float_to_registers(md)
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
    pub  = mb.findtext('pub-endpoint')
    pull = mb.findtext('pull-endpoint')
  else:
    pub  = 'tcp://127.0.0.1:5678'
    pull = 'tcp://127.0.0.1:1234'

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
