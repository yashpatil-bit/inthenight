# ES 245 Course Project, Part 2: Frequency-Domain Analysis (Group 4)

The instructor's message, task by task:

| Instructor asks | Task | Where it is done |
|---|---|---|
| Chirp signal as input, record the response | 5 | `arduino/Chirp_Signal_IR_modified` (the provided IR file, fixed) |
| "Real" transfer function with **n4sid** | 5 | `matlab/task5_identify.m` |
| Bode + **Nyquist** of the real TF → **gain and phase margins** | 6 | `matlab/task6_analysis.m` |
| Root locus, design criteria OS < 5 %, Ts < 5 s | 6 | `matlab/task6_analysis.m` |
| Which compensator: lead, lag or **lead-lag** | 7 | `matlab/task7_compensator.m` → **lead-lag** |
| Show the response with the compensator on the rig | 8 | `arduino/leadlag_controller` → `matlab/task8_hardware_eval.m` |

## Which of the two provided chirp files?

**`Chirp_Signal_IR.ino`.** Your rig (Part 1 report, Table 1) uses the Sharp
GP2Y0A21YK0F **IR** sensor on **A1** and the servo on **D3**, and the IR file
uses exactly those pins. `Chirp_Signal_Ultrasonic.ino` is for groups with an
HC-SR04 (trigger/echo pins), so ignore it.

The IR file cannot be used as it is:

| In the provided file | Problem | In `Chirp_Signal_IR_modified.ino` |
|---|---|---|
| `amplitude = 0` | no chirp at all, nothing to identify | 6° |
| `Serial.begin(9600)` | ~22 characters every 20 ms is more than 9600 baud carries, so the loop slows down and the timing is wrong | 115200 |
| open loop | ball = double integrator. Simulated: unless `centerAngle` is level to within ~1° and the swing is 2–6°, the ball hits the end in 2–5 s | optional weak PID (`HOLD_BALL = true`) adds a small correction to keep the ball on the beam |
| 0.05→5 Hz in 20 s | above ~2 Hz the ball moves < 0.1 mm/deg (below the noise); few cycles at low frequency | 0.1→2 Hz in 60 s, after 5 s settling |
| limits 60–180°, `centerAngle = 105` | 180° can jam the linkage; 105 is not your level angle | 60–120°, set `centerAngle` to *your* level angle |
| `servo.write()` | 1° steps | `writeMicroseconds`, ~0.1° steps |
| `lastMillis = now` | sample time drifts | fixed 20 ms grid |
| invalid reading → `-1` | breaks the model | last valid reading is kept |

The distance model (`35.7·V^-1.15`), 10-sample averaging, 20 ms sampling, the
linear chirp formula and the CSV columns `time_ms,servo_deg,dist_cm` are kept.
A 4th column `chirp_deg` (the chirp alone) is added. A light low-pass filter
(`FILTER_ALPHA = 0.3`) is used in both sketches, so the identified model
already contains its lag. Set `HOLD_BALL = false` to run the instructor's pure
open-loop test; then use an amplitude of 3–4° and a very well levelled beam.

## Key results to expect (and why)

1. **Units:** input = servo angle deviation from level (deg), output = ball
   distance (cm). The identified G(s) is in cm/deg, so the compensator numbers go
   into the Arduino sketch unchanged. In Part 1, the Simulink gains did not
   transfer to the rig because of units; this fixes that.
2. **n4sid:** with the ball-holding loop on, sensor noise feeds back into the
   servo signal and biases n4sid. `task5_identify` therefore uses the two-stage
   method (keep only the part of the servo signal caused by the chirp, then run
   n4sid). It block-averages the 50 Hz data to 10 Hz first: at 50 Hz all poles
   sit next to z = 1 and n4sid becomes ill-conditioned. It also fits the physical
   model `K·e^(−Td·s)/(s²(τs+1))` to the measured frequency response. Report
   both; Task 6 plots both.
3. **Margins:** the phase is at or below −180° over the useful band. The PM is
   about 0° (n4sid) or negative (physical model), and P control alone gives
   undamped oscillation, as in your Part 1 P video.
4. **Compensator = lead-lag.**
   - A **lead** is required: only a lead adds phase, and a lag alone can never
     stabilise this loop.
   - A **lag** with a pole at 0 (PI-type) is added for the steady state. A
     servo-level error or friction acts as a constant disturbance; lead alone
     leaves ~2 cm of error per degree, and the lag removes it. That is the
     Part 1 offset.
   - A **reference prefilter** cancels the lead zeros. Without it, set-point
     steps overshoot ~40 %.

---

## Before the lab (30 min, no hardware)

In MATLAB (**Control System** + **System Identification** toolboxes):
`cd matlab`, then `run_dry_test`. It runs Tasks 5–8 on
**simulated** logs (`sim/synthetic_data`) and writes the figures to
`dry_test_output/`. This checks your toolboxes. **Never use these results in the
report.**

## In the lab, step by step

### Step 1: level angle and sensor check (~20 min)
1. Upload `arduino/sensor_calibration`. In the Serial Monitor (115200 baud,
   newline), type angles (e.g. `88`, `92`) until the beam is **level**; use a phone
   spirit-level app. Put that angle in `centerAngle` (chirp sketch),
   `SERVO_CENTER` (controller sketch) and `P.centerAngle` (`matlab/config_part2.m`).
2. Optional but recommended: hold the ball at known distances (10, 12, …, 40 cm),
   note `mean_V`, fill in `matlab/calibrate_sensor.m` and run it. If the refit is
   clearly better than `35.7·V^-1.15`, put its two numbers in **both** sketches.

### Step 2: chirp test (Task 5, ~10 min)
1. Upload `arduino/Chirp_Signal_IR_modified`. Close the Serial Monitor.
2. In MATLAB, `cd matlab`, then:
   `log_serial_to_csv("COM5", "../data/chirp_run1.csv", 75)` (your port).
   Place the ball near 22 cm while the servo centres.
3. **First 5 s (no chirp):** the ball must be pulled towards 22 cm. If it is
   pushed away, set `DIRECTION = -1.0` in both sketches and `P.direction = -1`.
4. If the ball hits a beam end, lower `amplitude`.
5. Repeat as `chirp_run2.csv` (validation). **Film both.**

### Step 3: MATLAB (~5 min)
Run `task5_identify`, then `task6_analysis`, then `task7_compensator`. The last
script prints a block:
```
const float   KC           = 8.1776f;   // deg per cm
const int     N_LEAD       = 2;
...
```
Paste it over the same lines in `leadlag_controller.ino`. The values already
there come from the simulated rig, so they work as a start, but use yours.

### Step 4: compensator on the rig (Task 8, ~20 min)
1. Upload `leadlag_controller`. The ball should go to 22 cm and stay. Nudge it;
   it should come back.
2. Step run: `log_serial_to_csv("COM5", "../data/controller_steps.csv", 95, "s")`.
   The `"s"` starts automatic set-point steps of 17 ↔ 27 cm every 15 s.
3. Perturbation run: `log_serial_to_csv("COM5", "../data/controller_kick.csv", 40)`.
   Push the ball 3–4 times, a few seconds apart.
4. Run `task8_hardware_eval`. **Film both runs.**

| Symptom on the rig | Fix |
|---|---|
| Growing oscillation | Lower `KC` by 30 %, or set `P.PMmin = 50` and rerun Task 7 |
| Servo jitters / buzzes | Set `P.KcMax = 8` and rerun Task 7 |
| Ball pushed the wrong way | `DIRECTION` (both sketches) and `P.direction` |
| Ball settles off target, then creeps in | Friction + integral action; a better level angle helps |
| n4sid model looks odd | `P.designModel = 'structured'` (the default `'auto'` already picks the better one) |

---

## What goes into the report

- **Task 5:**
  - The experiment and the changes to the provided file (table above).
  - `task5_chirp_data.png`.
  - The n4sid orders and the chosen order, poles/zeros (`task5_pzmap.png`), the
    printed transfer function.
  - The structured fit (K, τ, Td).
  - Validation fit on run 2 (`task5_validation.png`).
  - Bode plot (`task5_bode.png`): −40 dB/dec magnitude; phase from −180° and
    falling (servo lag + delay).
  - Identified K versus the Part 1 theory (2.88 cm/(deg·s²); the ratio is
    printed).
- **Task 6, for the n4sid model (and the structured model as a check):**
  - Root locus `task6_rlocus_*.png`.
  - Nyquist `task6_nyquist_*.png` with the P, Z, N count.
  - Bode with PM/GM `task6_bode_*.png` (MATLAB also saves `task6_margin_*.png`).
  - Then the 6.3 table and `task6_existing_steps.png`: unity feedback fails, and
    the Part 1 PD/PID have large overshoot and small margins. This explains the
    Part 1 rig behaviour, so a compensator is needed.
- **Task 7:**
  - Why lead-lag (printout section A).
  - The design numbers (section C).
  - `task7_bode.png` (PM before → after).
  - `task7_step.png` (with and without prefilter).
  - `task7_disturbance.png` (why the lag), `task7_effort.png`.
  - Compensated root locus and Nyquist.
  - `task7_step_both_models.png` (the design also works on the n4sid model).
  - `task7_discrete.png` (20 ms Tustin = continuous).
- **Task 8:**
  - Implementation (Tustin sections, anti-windup, prefilter).
  - `task8_steps.png`, `task8_step_zoom.png` and the per-step OS/Ts/Tr/ess table.
    Metrics use 0.25 s smoothing because IR noise is about the size of the 5 % band.
  - `task8_perturbation.png` (peak and recovery time per push).
  - Comparison with the model (red dashed line) and with Part 1 (offset gone).
  - Limitations.

## Files

```
inthenight/
├── arduino/
│   ├── Chirp_Signal_IR_modified/   Task 5: the provided IR chirp file, fixed
│   ├── sensor_calibration/         level angle + voltage readings
│   └── leadlag_controller/         Task 8: lead^2 x PI-lag + prefilter, 20 ms
├── matlab/
│   ├── config_part2.m              all settings (level angle, file names, specs)
│   ├── task5_identify.m  task6_analysis.m  task7_compensator.m  task8_hardware_eval.m
│   ├── calibrate_sensor.m  log_serial_to_csv.m  run_dry_test.m
│   └── helpers: read_log, step_metrics, frf_estimate, fit_structured,
│                loop_margins, design_leadlag, leadlag_build
├── sim/ballbeam_sim.py             rig simulator running the sketch logic
│   └── synthetic_data/             SIMULATED logs, for testing only
└── data/                           put your real lab logs here
```

The MATLAB scripts ran end to end in GNU Octave (control package) on the
simulated logs. The MATLAB-only lines (n4sid with SSARX weighting, the `margin`
plot, `serialport`) follow the MathWorks documentation but were not run. The
sketches were compiled and run against a stub of the Arduino API, not on a board.
