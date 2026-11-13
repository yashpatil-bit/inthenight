%% config_part2.m  -  settings shared by every Part 2 script (edit only here)
% Units used everywhere in Part 2 (same as the Arduino sketches):
%   input  u = servo-command deviation from SERVO_CENTER, in degrees
%   output y = ball distance from the IR sensor, in cm
% The identified plant G(s) is therefore in cm/deg, and the compensator in
% deg/cm, so its numbers can be pasted into the Arduino sketch unchanged.

here      = fileparts(mfilename('fullpath'));
P.dataDir = fullfile(here, '..', 'data');      % put the logged CSV files here
P.figDir  = fullfile(here, '..', 'figures');   % figures for the report
P.resDir  = fullfile(here, '..', 'results');   % .mat files passed between tasks
if ~isempty(getenv('ES245_OUT_DIR'))           % test hook used with the simulator
    P.dataDir = getenv('ES245_OUT_DIR');
    P.figDir  = fullfile(P.dataDir, 'figures');
    P.resDir  = fullfile(P.dataDir, 'results');
end
if ~exist(P.figDir, 'dir'), mkdir(P.figDir); end
if ~exist(P.resDir, 'dir'), mkdir(P.resDir); end

P.isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
if P.isOctave, pkg load control; end

% ---- Arduino (must match the sketches) ----
P.Ts          = 0.02;         % sample time of BOTH sketches, s (instructor's 20 ms)
P.centerAngle = 90;           % centerAngle / SERVO_CENTER (level beam), deg
P.direction   = +1;           % DIRECTION in the sketches

% ---- Task 5: chirp data (Chirp_Signal_IR_modified.ino) ----
P.chirpFile    = 'chirp_run1.csv';   % estimation data
P.chirpValFile = 'chirp_run2.csv';   % validation run ('' if you only have one)
P.tChirp       = [5 65];             % s, sweep start/end = T_settle, T_settle+T_chirp
P.chirpBand    = [0.1 2.0];          % Hz, f0 and f1 of the sketch
P.idHold = true;                     % HOLD_BALL in the sketch, and its PID gains:
P.idKp = 0.6;  P.idKd = 0.8;  P.idTauD = 0.10;  P.idKi = 0.10;
P.orders       = 2:4;                % n4sid model orders to compare
P.TsId         = 0.1;                % s, data are block-averaged to this rate for n4sid
                                     % (at 50 Hz all poles crowd near z = 1 and the
                                     %  subspace fit becomes ill-conditioned)
P.forceOrder   = [];                 % e.g. 3 to override the automatic choice
P.designModel  = 'auto';            % 'auto' | 'structured' | 'n4sid' (model for Tasks 6-8)

% ---- Part 1 theoretical model, in Part 2 units (for comparison) ----
P.Ktheory = 1.65 * 100 * pi/180;     % 1.65 m/(rad s^2) = 2.88 cm/(deg s^2)

% ---- Task 6/7: design specifications ----
P.OSmax   = 5;      % percent overshoot
P.TsMax   = 5;      % s, settling time
P.band    = 0.05;   % settling band (5 %)
P.PMmin   = 40;     % deg, robustness floor used in the design search
P.GMmin   = 6;      % dB
P.KcMax   = 15;     % deg/cm, cap on the high-frequency controller gain (noise)

% ---- Task 8: hardware runs ----
P.stepFile  = 'controller_steps.csv';   % step mode run ("s" command)
P.kickFile  = 'controller_kick.csv';    % constant set point, ball nudged by hand
P.smoothSec = 0.25;                     % zero-phase smoothing for metrics, s
