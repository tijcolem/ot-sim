from __future__ import annotations

"""
Modbus TCP Meter Simulator - OT-sim module
Simulates a three-phase power meter, exposes readings via Modbus TCP,
and publishes them as Status messages on the OT-sim message bus.
"""

import math, random, signal, struct, sys, threading, time, typing

import otsim.msgbus.envelope as envelope
import xml.etree.ElementTree as ET

from otsim.msgbus.envelope import Point
from otsim.msgbus.pusher   import Pusher

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusDeviceContext,
    ModbusServerContext,
)
from pymodbus.server import StartTcpServer


def _float_to_registers(value: float) -> typing.List[int]:
    packed     = struct.pack('>f', float(value))
    high, low  = struct.unpack('>HH', packed)
    return [high, low]


class MeterSim:
  def __init__(self: MeterSim, pull: str, el: ET.Element):
    self.running = False

    self.name    = el.get('name', default='ot-sim-meter-sim')
    self.address = el.findtext('listen-address',  default='0.0.0.0')
    self.port    = int(el.findtext('port',         default='5020'))
    self.unit_id = int(el.findtext('unit-id',      default='1'))
    self.period  = float(el.findtext('update-interval', default='1'))

    tags = el.find('tags') or ET.Element('tags')

    self.tag_voltage_l1    = tags.findtext('voltage-l1',   default='meter.voltage-l1')
    self.tag_voltage_l2    = tags.findtext('voltage-l2',   default='meter.voltage-l2')
    self.tag_voltage_l3    = tags.findtext('voltage-l3',   default='meter.voltage-l3')
    self.tag_current_l1    = tags.findtext('current-l1',   default='meter.current-l1')
    self.tag_current_l2    = tags.findtext('current-l2',   default='meter.current-l2')
    self.tag_current_l3    = tags.findtext('current-l3',   default='meter.current-l3')
    self.tag_active_power  = tags.findtext('active-power', default='meter.active-power')
    self.tag_power_factor  = tags.findtext('power-factor', default='meter.power-factor')
    self.tag_frequency     = tags.findtext('frequency',    default='meter.frequency')
    self.tag_energy        = tags.findtext('energy',       default='meter.energy')

    pull_endpoint = el.findtext('pull-endpoint', default=pull)
    self.pusher   = Pusher(pull_endpoint)

    block        = ModbusSequentialDataBlock(0, [0] * 200)
    store        = ModbusDeviceContext(hr=block)
    self.context = ModbusServerContext(devices={self.unit_id: store}, single=False)


  def start(self: MeterSim):
    self.running = True

    threading.Thread(target=self._update_loop, daemon=True).start()
    threading.Thread(target=self._serve,       daemon=True).start()


  def stop(self: MeterSim):
    self.running = False


  def _update_loop(self: MeterSim):
    t          = 0
    energy_kwh = 0.0

    while self.running:
      t += 1
      time.sleep(self.period)

      v1 = 230.0 + random.uniform(-2, 2)
      v2 = 230.0 + random.uniform(-2, 2)
      v3 = 230.0 + random.uniform(-2, 2)

      i1 = 5.0 + 3.0 * math.sin(2 * math.pi * t / 60) + random.uniform(-0.2, 0.2)
      i2 = 5.0 + 3.0 * math.sin(2 * math.pi * t / 60 + 2.094) + random.uniform(-0.2, 0.2)
      i3 = 5.0 + 3.0 * math.sin(2 * math.pi * t / 60 + 4.189) + random.uniform(-0.2, 0.2)

      pf        = 0.95
      total_kw  = (v1 * i1 + v2 * i2 + v3 * i3) * pf / 1000
      freq      = 50.0 + random.uniform(-0.05, 0.05)
      energy_kwh += total_kw / 3600

      # update Modbus holding registers (float32, big-endian, two regs per value)
      registers = (
          _float_to_registers(v1)        + _float_to_registers(v2)        + _float_to_registers(v3)        +
          _float_to_registers(i1)        + _float_to_registers(i2)        + _float_to_registers(i3)        +
          _float_to_registers(total_kw)  + _float_to_registers(pf)        +
          _float_to_registers(freq)      + _float_to_registers(energy_kwh)
      )

      self.context[self.unit_id].setValues(3, 0, registers)

      # publish to OT-sim message bus
      points: typing.List[Point] = [
        {'tag': self.tag_voltage_l1,   'value': v1,         'ts': 0},
        {'tag': self.tag_voltage_l2,   'value': v2,         'ts': 0},
        {'tag': self.tag_voltage_l3,   'value': v3,         'ts': 0},
        {'tag': self.tag_current_l1,   'value': i1,         'ts': 0},
        {'tag': self.tag_current_l2,   'value': i2,         'ts': 0},
        {'tag': self.tag_current_l3,   'value': i3,         'ts': 0},
        {'tag': self.tag_active_power, 'value': total_kw,   'ts': 0},
        {'tag': self.tag_power_factor, 'value': pf,         'ts': 0},
        {'tag': self.tag_frequency,    'value': freq,       'ts': 0},
        {'tag': self.tag_energy,       'value': energy_kwh, 'ts': 0},
      ]

      env = envelope.new_status_envelope(self.name, {'measurements': points})
      self.pusher.push('RUNTIME', env)


  def _serve(self: MeterSim):
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
    pull = mb.findtext('pull-endpoint')
  else:
    pull = 'tcp://127.0.0.1:1234'

  modules: typing.List[MeterSim] = []

  for ms in root.findall('meter-sim'):
    module = MeterSim(pull, ms)
    module.start()

    modules.append(module)

  waiter = threading.Event()

  def handler(*_):
    waiter.set()

  signal.signal(signal.SIGINT, handler)
  waiter.wait()

  for module in modules:
    module.stop()
