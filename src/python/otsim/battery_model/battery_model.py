from __future__ import annotations

#####################################################################################################
###     Battery cell model in MATLAB by Shriram Santhanagopalan <Shriram.Santhanagopalan@nrel.gov>
###     Modified battery cell model in Python by Myungsoo Jun <Myungsoo.Jun@nrel.gov>
###     Adapted as an OT-sim module by wrapping the physics in the standard module pattern.
#####################################################################################################

import signal, sys, threading, time, typing

import numpy as np
import otsim.msgbus.envelope as envelope
import xml.etree.ElementTree as ET

from math import sqrt, log, floor
from scipy.interpolate import pchip_interpolate

from otsim.helics_helper     import DataType, HelicsFederate, Publication, Subscription
from otsim.msgbus.envelope   import Envelope, Point
from otsim.msgbus.pusher     import Pusher
from otsim.msgbus.subscriber import Subscriber


# ── Battery parameters ────────────────────────────────────────────────────────

_OCVpts = [3.0721, 3.7123, 3.7734, 3.7942, 3.8004, 3.8316, 3.8711, 3.8946, 3.9247, 3.9845, 4.0907]
_SOCpts = np.linspace(0, 1, num=len(_OCVpts))

_CellAh   = 26 * 3600
_BattVolt = 400
_BattSize = 660
_Ns       = round(_BattVolt / pchip_interpolate(_SOCpts, _OCVpts, 0.5))
_Np       = floor(_BattSize * 1.0e3 / _BattVolt / (_CellAh / 3600))

_xnl = 0.001
_xnu = 0.9372
_xpl = 0.5777
_xpu = 0.999


def _upos(x: float) -> float:
    X  = x
    a1 =     6.465320913090
    a2 =    -8.059044434060
    a3 =     8.595201030710
    a4 =    -3.061444018620
    a5 = -4262.994775810
    a6 =    43.24510260320
    a7 =  -462.2750487970
    a8 =  4294.399910710

    x1 = 0.9816662754160
    x2 = 0.6319306804190
    x3 = 0.5632963031200
    x4 = 0.5167224120090

    if x2 < X and X <= x1:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00
    elif X > x1:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a5*(X-x1)**3.00
    elif x3 < X and X <= x2:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a6*(X-x2)**3.00
    elif x4 < X and X <= x3:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a6*(X-x2)**3.00 + a7*(X-x3)**3.00
    elif X <= x4:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a6*(X-x2)**3.00 + a7*(X-x3)**3.00 + a8*(X-x4)**3.00

    return dumy


def _uneg(x: float) -> float:
    X  = x
    a1 =      1.331269097090 + 0.5162746e-01
    a2 =     -5.843671146280
    a3 =      8.819628322980
    a4 =     -4.449264481680
    a5 =     29.54504805800
    a6 =    -33.12358024580
    a7 =     99.13821710150
    a8 =   -828.7522015810
    a9 = -73115.58086430

    x1 = 0.50
    x2 = 0.4350951654950
    x3 = 0.1480397512090
    x4 = 0.910366871229e-1
    x5 = 0.134322870209e-1

    if X > x1:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00
    elif x2 < X and X <= x1:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a5*(X-x1)**3.00
    elif x3 < X and X <= x2:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a5*(X-x1)**3.00 + a6*(X-x2)**3.00
    elif x4 < X and X <= x3:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a5*(X-x1)**3.00 + a6*(X-x2)**3.00 + a7*(X-x3)**3.00
    elif x5 < X and X <= x4:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a5*(X-x1)**3.00 + a6*(X-x2)**3.00 + a7*(X-x3)**3.00 + a8*(X-x4)**3.00
    elif X <= x5:
        dumy = a1 + a2*X + a3*X**2.00 + a4*X**3.00 + a5*(X-x1)**3.00 + a6*(X-x2)**3.00 + a7*(X-x3)**3.00 + a8*(X-x4)**3.00 + a9*(X-x5)**3.00

    return dumy


def _battery_cell(
    f_curr: float, fx0p: float, fx0n: float, fdt: float
) -> typing.List[float]:
    x0p   = fx0p
    x0n   = fx0n
    epsp  = 0.39
    epsn  = 0.385
    epspf = 0.03
    epsnf = 0.024
    csmaxp = 51410
    csmaxn = 31833

    Rp  = 8.5e-6
    Rn  = 12.5e-6
    kn  = 1.764e-11
    kp  = 6.6667e-11
    Dsp = epsp * 2.5641e-14
    Dsn = epsn * 8.8312e-14
    ce  = 1000
    F   = 96485
    R   = 8.314
    T   = 298.15
    Rcell = 0.0012
    iapp  = f_curr
    dt    = fdt

    Vp = 3 * 1.58967 * 70e-6
    Vn = 3 * 1.0824  * 73.5e-6

    Sp = 3 / Rp * (1 - epsp - epspf) * Vp
    Sn = 3 / Rn * (1 - epsn - epsnf) * Vn
    Jp =  iapp / F / Sp
    Jn = -iapp / F / Sn

    deltap = -Jp * Rp / Dsp / csmaxp
    deltan = -Jn * Rn / Dsn / csmaxn

    lambda1 = 4.4934095
    lambda2 = 7.7252518
    lambda3 = 10.90412166
    lambda4 = 14.0661939
    lambda5 = 17.22075527

    t = fdt

    xps = x0p + deltap * (
          (3*Dsp/Rp**2*(t+dt) - 2*(
            (1/lambda1**2*(1-lambda1**2*Dsp*(t+dt)/Rp**2))+
            (1/lambda2**2*(1-lambda2**2*Dsp*(t+dt)/Rp**2))+
            (1/lambda3**2*(1-lambda3**2*Dsp*(t+dt)/Rp**2))+
            (1/lambda4**2*(1-lambda4**2*Dsp*(t+dt)/Rp**2))+
            (1/lambda5**2*(1-lambda5**2*Dsp*(t+dt)/Rp**2))))
         -(3*Dsp/Rp**2*t - 2*(
            (1/lambda1**2*(1-lambda1**2*Dsp*t/Rp**2))+
            (1/lambda2**2*(1-lambda2**2*Dsp*t/Rp**2))+
            (1/lambda3**2*(1-lambda3**2*Dsp*t/Rp**2))+
            (1/lambda4**2*(1-lambda4**2*Dsp*t/Rp**2))+
            (1/lambda5**2*(1-lambda5**2*Dsp*t/Rp**2)))))

    xns = x0n + deltan * (
          (3*Dsn/Rn**2*(t+dt) - 2*(
            (1/lambda1**2*(1-lambda1**2*Dsn*(t+dt)/Rn**2))+
            (1/lambda2**2*(1-lambda2**2*Dsn*(t+dt)/Rn**2))+
            (1/lambda3**2*(1-lambda3**2*Dsn*(t+dt)/Rn**2))+
            (1/lambda4**2*(1-lambda4**2*Dsn*(t+dt)/Rn**2))+
            (1/lambda5**2*(1-lambda5**2*Dsn*(t+dt)/Rn**2))))
         -(3*Dsn/Rn**2*t - 2*(
            (1/lambda1**2*(1-lambda1**2*Dsn*t/Rn**2))+
            (1/lambda2**2*(1-lambda2**2*Dsn*t/Rn**2))+
            (1/lambda3**2*(1-lambda3**2*Dsn*t/Rn**2))+
            (1/lambda4**2*(1-lambda4**2*Dsn*t/Rn**2))+
            (1/lambda5**2*(1-lambda5**2*Dsn*t/Rn**2)))))

    Up = _upos(xps)
    mp = iapp / F / kp / Sp / csmaxp / sqrt((ce) * (1 - xps) * xps)

    Un = _uneg(xns)
    mn = iapp / F / kn / Sn / csmaxn / sqrt((ce) * (1 - xns) * xns)

    Vcell = (Up - Un
             + 2*R*T/F * log((sqrt(mp*mp + 4) + mp) / 2)
             + 2*R*T/F * log((sqrt(mn*mn + 4) + mn) / 2)
             + iapp * Rcell)

    xp    = xps
    xn    = xns
    i_out = f_curr
    v_out = Vcell
    q_out = (v_out - (Up - Un)) * i_out

    return [xp, xn, q_out, i_out, v_out]


def _xn2Soc(xn: float) -> float:
    return (xn - _xnl) / (_xnu - _xnl)


def _soc2xpn(soc: float) -> typing.List[float]:
    xp = _xpu + soc * (_xpl - _xpu)
    xn = _xnl + soc * (_xnu - _xnl)
    return [xp, xn]


def _battery_pack_model(
    current_input: float, soc_init: float, dt: float
) -> typing.List[float]:
    cell_current   = current_input / _Np
    [x0p, x0n]    = _soc2xpn(soc_init)
    [xp, xn, q_out_cell, i_out_cell, v_out_cell] = _battery_cell(cell_current, x0p, x0n, dt)
    v_out = v_out_cell * _Ns
    i_out = i_out_cell * _Np
    soc   = _xn2Soc(xn)
    return [i_out, v_out, soc]


# ── OT-sim module ─────────────────────────────────────────────────────────────

class BatteryModel:
  def __init__(self: BatteryModel, pub: str, pull: str, el: ET.Element):
    # mutex protects access to self.current_cmd across threads
    self.mutex   = threading.Lock()
    self.running = False

    self.name = el.get('name', default='ot-sim-battery-model')

    self.soc = float(el.findtext('initial-soc', default='0.5'))
    self.dt  = float(el.findtext('dt',          default='1'))

    self.current_cmd     = 0.0
    self.current_tag     = el.findtext('current-tag')

    self.voltage_tag     = el.findtext('voltage-tag',     default='battery.voltage')
    self.current_out_tag = el.findtext('current-out-tag', default='battery.current')
    self.soc_tag         = el.findtext('soc-tag',         default='battery.soc')

    pub_endpoint  = el.findtext('pub-endpoint',  default=pub)
    pull_endpoint = el.findtext('pull-endpoint', default=pull)

    self.pusher = Pusher(pull_endpoint)

    if self.current_tag:
      self.subscriber = Subscriber(pub_endpoint)
      # handle_update: receives current from Update envelopes (direct control commands)
      # handle_status: receives current from Status envelopes published by the IO module
      #                when it bridges OpenDSS HELICS subscriptions onto the msgbus
      self.subscriber.add_update_handler(self.handle_update)
      self.subscriber.add_status_handler(self.handle_status)
    else:
      self.subscriber = None


  def start(self: BatteryModel):
    self.running = True

    if self.subscriber:
      self.subscriber.start('RUNTIME')

    threading.Thread(target=self.run, daemon=True).start()


  def stop(self: BatteryModel):
    self.running = False

    if self.subscriber:
      self.subscriber.stop()


  def handle_update(self: BatteryModel, env: Envelope):
    update = envelope.update_from_envelope(env)

    if update:
      for point in update['updates']:
        if self.current_tag and point['tag'] == self.current_tag:
          with self.mutex:
            self.current_cmd = float(point['value'])


  def handle_status(self: BatteryModel, env: Envelope):
    '''Receive current command published as a Status envelope by the IO module.

    The IO module's action_subscriptions() pushes a Status envelope onto the
    msgbus for every HELICS time grant.  Handling it here drives one battery
    step per OpenDSS time step, matching the power_output.py pattern.
    '''
    status = envelope.status_from_envelope(env)

    if status:
      for point in status.get('measurements', []):
        if self.current_tag and point['tag'] == self.current_tag:
          with self.mutex:
            self.current_cmd = float(point['value'])
          self._do_step()
          break


  def _do_step(self: BatteryModel):
    '''Advance the battery model by one dt and publish results onto the msgbus.'''
    with self.mutex:
      current = self.current_cmd

    try:
      [i_out, v_out, soc] = _battery_pack_model(current, self.soc, self.dt)
      self.soc = soc
    except Exception as exc:
      print(f'[{self.name}] battery model error: {exc}')
      return

    points: typing.List[Point] = [
      {'tag': self.voltage_tag,     'value': v_out, 'ts': 0},
      {'tag': self.current_out_tag, 'value': i_out, 'ts': 0},
      {'tag': self.soc_tag,         'value': soc,   'ts': 0},
    ]

    env = envelope.new_status_envelope(self.name, {'measurements': points})
    self.pusher.push('RUNTIME', env)

    env = envelope.new_update_envelope(self.name, {'updates': points})
    self.pusher.push('RUNTIME', env)


  def run(self: BatteryModel):
    # When current_tag is configured a subscriber is active and battery steps
    # are driven by incoming messages (handle_status for IO/HELICS, handle_update
    # for direct control).  Fall back to a wall-clock timer only in standalone
    # mode where no external current source is wired up.
    if self.subscriber:
      return

    while self.running:
      self._do_step()
      time.sleep(self.dt)


# ── HELICS co-simulation federate ─────────────────────────────────────────────

class BatteryModelFederate(HelicsFederate):
  """Battery model that synchronises with an OpenDSS co-simulation via HELICS.

  Each HELICS time grant corresponds to one battery model time step (dt).
  The current command is read from a HELICS subscription published by OpenDSS.
  Battery outputs (voltage, current, SOC) are published back to HELICS and
  also pushed onto the OT-sim message bus as a Status envelope.

  XML configuration example::

    <battery-model name="battery-helics">
      <initial-soc>0.5</initial-soc>
      <current-tag>battery.current.cmd</current-tag>
      <voltage-tag>battery.voltage</voltage-tag>
      <current-out-tag>battery.current</current-out-tag>
      <soc-tag>battery.soc</soc-tag>
      <helics>
        <broker-endpoint>127.0.0.1</broker-endpoint>
        <federate-name>battery-model</federate-name>
        <federate-log-level>SUMMARY</federate-log-level>
        <start-time>1</start-time>
        <end-time>3600</end-time>
        <step-time>60</step-time>
        <real-time>false</real-time>
        <!-- tag must match current-tag above -->
        <subscription key="OpenDSS/battery.current.cmd" type="double" tag="battery.current.cmd"/>
        <!-- tags must match voltage-tag / current-out-tag / soc-tag above -->
        <publication key="battery.voltage"  type="double" tag="battery.voltage"/>
        <publication key="battery.current"  type="double" tag="battery.current"/>
        <publication key="battery.soc"      type="double" tag="battery.soc"/>
      </helics>
    </battery-model>
  """

  @staticmethod
  def _parse_type(typ: str) -> DataType:
    if typ == 'boolean':
      return DataType.boolean
    return DataType.double


  def __init__(self: 'BatteryModelFederate', pub: str, pull: str, el: ET.Element):
    self.name = el.get('name', default='ot-sim-battery-model-helics')

    self.soc         = float(el.findtext('initial-soc', default='0.5'))
    self.current_tag = el.findtext('current-tag')

    self.voltage_tag     = el.findtext('voltage-tag',     default='battery.voltage')
    self.current_out_tag = el.findtext('current-out-tag', default='battery.current')
    self.soc_tag         = el.findtext('soc-tag',         default='battery.soc')

    # Battery state – updated each time step by action_post_request_time.
    self.current_cmd = 0.0
    self.v_out       = 0.0
    self.i_out       = 0.0

    # HELICS key → OT-sim tag mappings (built from <subscription>/<publication>).
    self.sub_keys: typing.Dict[str, str] = {}
    self.pub_keys: typing.Dict[str, str] = {}

    helics_el = el.find('helics')
    assert helics_el is not None, '<helics> element required for co-simulation mode'

    broker        = helics_el.findtext('broker-endpoint',    default='127.0.0.1')
    log_level     = helics_el.findtext('federate-log-level', default='SUMMARY')
    federate_name = helics_el.findtext('federate-name',      default=self.name)
    start         = int(helics_el.findtext('start-time',     default='1'))
    end           = int(helics_el.findtext('end-time',       default='3600'))
    step          = int(helics_el.findtext('step-time',      default='60'))
    real_time_str = helics_el.findtext('real-time',          default='false')

    self.dt = float(step)

    HelicsFederate.federate_name                  = federate_name
    HelicsFederate.federate_info_core_init_string = f'--federates=1 --broker={broker} --loglevel={log_level}'
    HelicsFederate.federate_info_log_level        = log_level
    HelicsFederate.federate_info_real_time        = real_time_str.lower() in ['true', 'yes', '1']
    HelicsFederate.start_time                     = start
    HelicsFederate.end_time                       = end
    HelicsFederate.step_time                      = step
    HelicsFederate.subscriptions                  = []
    HelicsFederate.publications                   = []

    for s in helics_el.findall('subscription'):
      key = s.findtext('key')
      typ = BatteryModelFederate._parse_type(s.findtext('type', default='double'))
      tag = s.findtext('tag') or key.split('/')[-1]

      HelicsFederate.subscriptions.append(Subscription(key, typ))
      self.sub_keys[key] = tag

    for p in helics_el.findall('publication'):
      key = p.findtext('key')
      typ = BatteryModelFederate._parse_type(p.findtext('type', default='double'))
      tag = p.findtext('tag') or key

      HelicsFederate.publications.append(Publication(key, typ))
      self.pub_keys[key] = tag

    HelicsFederate.__init__(self, module_name=self.name)

    pub_endpoint  = helics_el.findtext('pub-endpoint',  default=pub)
    pull_endpoint = helics_el.findtext('pull-endpoint', default=pull)

    self.pusher = Pusher(pull_endpoint)


  def start(self: 'BatteryModelFederate'):
    threading.Thread(target=self.run, daemon=True).start()


  def stop(self: 'BatteryModelFederate'):
    pass  # HELICS run loop terminates naturally when end_time is reached.


  def action_subscriptions(self: 'BatteryModelFederate', data: typing.Dict, ts: float):
    """Read new current command from HELICS subscriptions."""
    for key, value in data.items():
      if value is None:
        continue
      tag = self.sub_keys.get(key)
      if tag and tag == self.current_tag:
        self.current_cmd = float(value)


  def action_publications(self: 'BatteryModelFederate', data: typing.Dict, ts: float):
    """Populate HELICS publications with the latest battery outputs."""
    for key, tag in self.pub_keys.items():
      if tag == self.voltage_tag:
        data[key] = self.v_out
      elif tag == self.current_out_tag:
        data[key] = self.i_out
      elif tag == self.soc_tag:
        data[key] = self.soc


  def action_post_request_time(self: 'BatteryModelFederate'):
    """Advance the battery model by one HELICS time step and push results to msgbus."""
    try:
      [i_out, v_out, soc] = _battery_pack_model(self.current_cmd, self.soc, self.dt)
      self.soc   = soc
      self.v_out = v_out
      self.i_out = i_out
    except Exception as exc:
      print(f'[{self.name}] battery model error: {exc}')
      return

    points: typing.List[Point] = [
      {'tag': self.voltage_tag,     'value': self.v_out, 'ts': 0},
      {'tag': self.current_out_tag, 'value': self.i_out, 'ts': 0},
      {'tag': self.soc_tag,         'value': self.soc,   'ts': 0},
    ]

    env = envelope.new_status_envelope(self.name, {'measurements': points})
    self.pusher.push('RUNTIME', env)

    env = envelope.new_update_envelope(self.name, {'updates': points})
    self.pusher.push('RUNTIME', env)


  def action_endpoints_send(self: 'BatteryModelFederate', endpoints: typing.Dict, ts: float):
    pass


  def action_endpoints_recv(self: 'BatteryModelFederate', endpoints: typing.Dict, ts: float):
    pass


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

  modules: typing.List[typing.Union[BatteryModel, BatteryModelFederate]] = []

  for bm in root.findall('battery-model'):
    if bm.find('helics') is not None:
      module: typing.Union[BatteryModel, BatteryModelFederate] = BatteryModelFederate(pub, pull, bm)
    else:
      module = BatteryModel(pub, pull, bm)
    module.start()

    modules.append(module)

  waiter = threading.Event()

  def handler(*_):
    waiter.set()

  signal.signal(signal.SIGINT, handler)
  waiter.wait()

  for module in modules:
    module.stop()
