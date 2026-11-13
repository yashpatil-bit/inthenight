%% run_dry_test.m  -  try the whole MATLAB pipeline on SIMULATED data
% Run this once BEFORE the lab to check that your MATLAB installation has
% everything (Control System + System Identification toolboxes) and to see
% what each script prints and plots. It copies the synthetic logs from
% sim/synthetic_data into a temporary folder under the names the scripts
% expect, runs Tasks 5-8 there, and leaves your real data/ folder untouched.
%
% NOTHING produced by this script is a measurement. Do not use it in the report.
here = fileparts(mfilename('fullpath'));
src  = fullfile(here, '..', 'sim', 'synthetic_data');
tmp  = fullfile(here, '..', 'dry_test_output');
if ~exist(tmp, 'dir'), mkdir(tmp); end
copyfile(fullfile(src, 'SYNTHETIC_chirp.csv'),            fullfile(tmp, 'chirp_run1.csv'));
copyfile(fullfile(src, 'SYNTHETIC_chirp_run2.csv'),       fullfile(tmp, 'chirp_run2.csv'));
copyfile(fullfile(src, 'SYNTHETIC_controller_steps.csv'), fullfile(tmp, 'controller_steps.csv'));
copyfile(fullfile(src, 'SYNTHETIC_controller_kick.csv'),  fullfile(tmp, 'controller_kick.csv'));
setenv('ES245_OUT_DIR', tmp);
try
    task5_identify;
    task6_analysis;
    task7_compensator;
    task8_hardware_eval;
catch err
    setenv('ES245_OUT_DIR', '');          % never leave the redirect switched on
    rethrow(err);
end
setenv('ES245_OUT_DIR', '');
fprintf('\nDry test finished. Figures are in %s\n', fullfile(tmp, 'figures'));
