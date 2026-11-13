"""Hardware-realistic ball-and-beam simulator for ES 245 Part 2 (pre-lab checks).

It mirrors the two Arduino sketches line by line so that they can be tried
before going to the lab:

  python3 ballbeam_sim.py chirp [seed] [file]
        runs Chirp_Signal_IR_modified.ino logic, writes synthetic_data/<file>
        (default SYNTHETIC_chirp.csv; use seed 2 + SYNTHETIC_chirp_run2.csv
        for a second, independent run)
  python3 ballbeam_sim.py control [KC N_LEAD LEAD_Z LEAD_P USE_LAG LAG_Z LAG_P]
        runs leadlag_controller.ino logic: a step-mode run and a run where the
        ball is flicked, writes synthetic_data/SYNTHETIC_controller_*.csv

Requires numpy only.

The plant deliberately contains effects that are NOT in G(s) = K/s^2:
  * the nonlinear four-bar map alpha(theta) of the Part 1 report (Eq. 11),
  * a level-beam command of 92 deg while the sketch assumes 90 deg (offset),
  * servo lag (first order, 60 ms) and rate limit, 20 ms PWM refresh,
  * rolling resistance and a little static friction, beam end stops,
  * the Sharp sensor's 1/L curve (slightly different from the assumed
    calibration), 38 ms update period, noise and 10-bit quantisation.

THE CSV FILES IT WRITES ARE SYNTHETIC. They only exist to test the MATLAB
pipeline; never put them in the report as measurements.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "synthetic_data")

# ------------------------------------------------------------- true plant ---
L1, L2, L3, L4, BETA = 250.0, 100.0, 55.0, 285.0, math.radians(26.5)  # mm
THETA0 = math.radians(43.48)          # crank angle for a level beam
LEVEL_CMD_TRUE = 92.0                 # servo command that really levels the beam
SERVO_TAU, SERVO_RATE = 0.06, 300.0   # s, deg/s
G = 9.81
MU_ROLL, MU_STATIC = 0.002, 0.003
BEAM_MIN, BEAM_MAX = 8.0, 48.0        # cm from the sensor (end stops)
NOISE_V = 0.012
SENS_A_TRUE, SENS_B_TRUE = 34.5, -1.20    # real sensor: L = A V^B (sketch assumes 35.7, -1.15)
SENSOR_PERIOD = 0.038


def alpha_of_theta(th):
    """Part 1 Eq. (11): beam angle for crank angle th (rad)."""
    px = L4 * math.cos(BETA) + L3 * math.cos(th)
    py = -L4 * math.sin(BETA) + L3 * math.sin(th)
    p = math.hypot(px, py)
    arg = (L1**2 + p**2 - L2**2) / (2 * L1 * p)
    return math.atan2(py, px) + math.acos(max(-1.0, min(1.0, arg)))


ALPHA0 = alpha_of_theta(THETA0)       # ~0, removed so level means alpha = 0


class Plant:
    def __init__(self, dist0=22.0, seed=1):
        self.rng = np.random.default_rng(seed)
        self.x, self.v = dist0, 0.0           # ball distance from sensor (cm, cm/s)
        self.servo = 90.0                     # actual servo angle (deg)
        self.pwm_cmd = 90.0                   # command latched by the 50 Hz PWM
        self.cmd = 90.0
        self.t = 0.0
        self.next_pwm = 0.0
        self.next_sensor = 0.0
        self.v_out = self._volts()

    def _volts(self):
        v = (self.x / SENS_A_TRUE) ** (1.0 / SENS_B_TRUE) + self.rng.normal(0, NOISE_V)
        return min(max(v, 0.0), 3.2)

    def adc(self):
        return int(round(self.v_out / 5.0 * 1023))

    def step(self, dt):
        if self.t >= self.next_pwm:                        # PWM frame
            self.pwm_cmd, self.next_pwm = self.cmd, self.next_pwm + 0.02
        if self.t >= self.next_sensor:                     # sensor update
            self.v_out, self.next_sensor = self._volts(), self.next_sensor + SENSOR_PERIOD
        rate = (self.pwm_cmd - self.servo) / SERVO_TAU
        self.servo += max(-SERVO_RATE, min(SERVO_RATE, rate)) * dt
        th = THETA0 + math.radians(self.servo - LEVEL_CMD_TRUE)
        a_g = 5.0 / 7.0 * G * math.sin(alpha_of_theta(th) - ALPHA0) * 100.0  # cm/s^2
        if abs(self.v) < 0.3 and abs(a_g) < MU_STATIC * G * 100:
            self.v, acc = 0.0, 0.0                         # stuck by static friction
        else:
            acc = a_g - math.copysign(MU_ROLL * G * 100, self.v if self.v else a_g)
        self.v += acc * dt
        self.x += self.v * dt
        if self.x < BEAM_MIN or self.x > BEAM_MAX:
            self.x, self.v = min(max(self.x, BEAM_MIN), BEAM_MAX), 0.0
        self.t += dt


# ------------------------------------------------- Arduino code, ported ---
T_S, SERVO_CENTER, SERVO_MIN, SERVO_MAX, DIRECTION = 0.02, 90.0, 60.0, 120.0, 1.0
A_CAL, B_CAL, DIST_MIN, DIST_MAX, FILTER_ALPHA = 35.7, -1.15, 4.0, 48.0, 0.3
_last = [22.0]


def angle_quant(deg):            # writeMicroseconds resolution
    us = round(544 + deg * (2400 - 544) / 180)
    return (us - 544) * 180 / (2400 - 544)


def volts_to_cm(v):                     # Chirp_Signal_IR model, hold last valid
    if v <= 0:
        return _last[0]
    d = A_CAL * v ** B_CAL
    if DIST_MIN <= d <= DIST_MAX:
        _last[0] = d
    return _last[0]


def run(controller, t_end, dist0=22.0, seed=1, kick_at=None):
    plant = Plant(dist0, seed)
    dt = 0.001
    for _ in range(1500):                      # 1.5 s hold at centre
        plant.cmd = SERVO_CENTER
        plant.step(dt)
    v_filt = plant.adc() * 5 / 1023
    controller.init(volts_to_cm(v_filt))
    rows, t = [], 0.0
    while t < t_end - 1e-9:
        v_raw = plant.adc() * 5 / 1023
        v_filt = FILTER_ALPHA * v_raw + (1 - FILTER_ALPHA) * v_filt
        y = volts_to_cm(v_filt)
        u, extra = controller.update(t, y)
        cmd = min(max(SERVO_CENTER + DIRECTION * u, SERVO_MIN), SERVO_MAX)
        u = (cmd - SERVO_CENTER) / DIRECTION
        controller.saturated(u)
        plant.cmd = angle_quant(cmd)
        rows.append([round(t * 1000), v_filt, y, u, *extra, plant.x])
        for _ in range(int(round(T_S / dt))):
            plant.step(dt)
        t += T_S
        if kick_at is not None and any(abs(t - k) < T_S / 2 for k in np.atleast_1d(kick_at)):
            plant.v += 15.0                    # flick the ball: +15 cm/s
    return np.array(rows)


class ChirpPD:
    """Chirp_Signal_IR_modified.ino: linear chirp + weak ball-holding PID."""
    REF, KP, KD, TAU_D, KI, I_LIMIT = 22.0, 0.6, 0.8, 0.10, 0.10, 15.0
    AMP, F0, F1, T_SETTLE, T_CHIRP, T_TAIL = 6.0, 0.10, 2.0, 5.0, 60.0, 3.0
    HOLD = True

    def init(self, y0):
        self.y_prev, self.d, self.i = y0, 0.0, 0.0

    def chirp(self, t):
        if t < 0 or t > self.T_CHIRP:
            return 0.0
        k = (self.F1 - self.F0) / self.T_CHIRP
        return self.AMP * math.sin(2 * math.pi * (self.F0 * t + 0.5 * k * t * t))

    def update(self, t, y):
        self.d = (self.TAU_D * self.d + (y - self.y_prev)) / (self.TAU_D + T_S)
        self.y_prev = y
        self.i = min(max(self.i + self.KI * (self.REF - y) * T_S, -self.I_LIMIT), self.I_LIMIT)
        c = self.chirp(t - self.T_SETTLE)
        hold = self.KP * (self.REF - y) - self.KD * self.d + self.i if self.HOLD else 0.0
        return hold + c, [c]

    def saturated(self, u):
        pass

    @property
    def duration(self):
        return self.T_SETTLE + self.T_CHIRP + self.T_TAIL


class Section:
    def __init__(self, n1, n0, p, x0):
        k = 2.0 / T_S
        d = k + p
        self.b0, self.b1, self.a1 = (n1 * k + n0) / d, (n0 - n1 * k) / d, (p - k) / d
        self.x1, self.y1 = x0, (x0 * n0 / p if p > 0 else 0.0)

    def step(self, x):
        y = self.b0 * x + self.b1 * self.x1 - self.a1 * self.y1
        self.x1, self.y1 = x, y
        return y


class LeadLag:
    def __init__(self, kc, n_lead, lz, lp, use_lag, gz, gp, use_pre=True,
                 lag_limit_deg=15.0, ref_low=17.0, ref_high=27.0, period=15.0):
        self.__dict__.update(locals())
        self.was_sat = False

    def init(self, y0):
        self.lead = [Section(1, self.lz, self.lp, 0) for _ in range(self.n_lead)]
        self.pre = [Section(0, self.lz, self.lz, y0) for _ in range(self.n_lead)]
        self.lag = Section(1, self.gz, self.gp, 0)

    def update(self, t, y):
        ref = self.ref_low if int(t // self.period) % 2 == 0 else self.ref_high
        rf = ref
        if self.use_pre:
            for s in self.pre:
                rf = s.step(rf)
        e = rf - y
        v = e
        for s in self.lead:
            v = s.step(v)
        w = v
        if self.use_lag:
            old_x1, old_y1 = self.lag.x1, self.lag.y1
            w = self.lag.step(v)
            if self.was_sat:                                # conditional integration
                self.lag.x1, self.lag.y1 = v, v + (old_y1 - old_x1)
                w = self.lag.y1
            lim = self.lag_limit_deg / self.kc
            w = v + min(max(w - v, -lim), lim)
            self.lag.y1 = w
        return self.kc * w, [ref, rf]

    def saturated(self, u):
        self.was_sat = abs(u) >= 30.0 - 1e-6


def save(path, rows, header):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savetxt(path, rows, delimiter=",", header=header, comments="", fmt="%.4f")
    print("wrote", os.path.relpath(path))


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "chirp"
    if mode == "chirp":
        seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        name = sys.argv[3] if len(sys.argv) > 3 else "SYNTHETIC_chirp.csv"
        c = ChirpPD()
        r = run(c, c.duration, seed=seed)
        print("ball range during test: %.1f .. %.1f cm, servo u: %.1f .. %.1f deg"
              % (r[:, 5].min(), r[:, 5].max(), r[:, 3].min(), r[:, 3].max()))
        out = np.column_stack([r[:, 0], SERVO_CENTER + r[:, 3], r[:, 2], r[:, 4]])
        save(os.path.join(DATA, name), out, "time_ms,servo_deg,dist_cm,chirp_deg")
    else:
        # numbers as printed by task7_compensator.m (edit to try other designs)
        args = [float(a) for a in sys.argv[2:]] if len(sys.argv) > 2 else \
            [8.1776, 2, 1.2, 4.8, 1, 0.6, 0.0]
        kc, n, lz, lp, ul, gz, gp = args
        # 1) automatic set-point steps 17 <-> 27 cm (the "s" command)
        r = run(LeadLag(kc, int(n), lz, lp, bool(ul), gz, gp), 92.0, dist0=30.0)
        save(os.path.join(DATA, "SYNTHETIC_controller_steps.csv"), r[:, :6],
             "t_ms,volt,dist_cm,u_deg,ref_cm,reff_cm")
        # 2) constant set point 22 cm, ball flicked at 10 s and 25 s
        ctl = LeadLag(kc, int(n), lz, lp, bool(ul), gz, gp, ref_low=22.0, ref_high=22.0)
        r2 = run(ctl, 40.0, dist0=22.0, kick_at=(10.0, 25.0))
        save(os.path.join(DATA, "SYNTHETIC_controller_kick.csv"), r2[:, :6],
             "t_ms,volt,dist_cm,u_deg,ref_cm,reff_cm")
        print("servo u range in step run: %.1f .. %.1f deg" % (r[:, 3].min(), r[:, 3].max()))
