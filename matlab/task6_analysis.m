%% task6_analysis.m  -  Task 6: root locus, Nyquist, Bode and design criteria
% Uses the plants identified in Task 5 (results/plant_model.mat):
%   Gn4 - the n4sid "real transfer function" (what the instructor asks for)
%   Gst - the physically structured fit K e^(-Td s)/(s^2 (tau s + 1))
% Both are analysed so the report can show that they agree.
% 6.1 root locus and Nyquist of the open loop  (C = K, proportional)
% 6.2 Bode plot, gain and phase margins, link to 6.1
% 6.3 the existing system against OS < 5 %, Ts < 5 s (5 % band):
%     unity feedback, and the Part 1 hardware PD/PID gains converted to cm units
% Output: figures/task6_*_<model>.png and a summary in the command window.
config_part2;
load(fullfile(P.resDir, 'plant_model.mat'), 'G', 'Gn4', 'Gst', 'modelName');
s = tf('s');
models = {Gn4, 'n4sid'; Gst, 'structured'};

for im = 1:size(models, 1)
    Gm = models{im, 1};  tag = models{im, 2};
    fprintf('\n================ %s model ================\n', tag);
    Gm

    %% ---- 6.1 root locus ----
    pG = pole(Gm);
    fprintf('open-loop poles: %s\n', num2str(pG.', 4));
    fprintf('open-loop zeros: %s\n', num2str(zero(Gm).', 4));
    figure; rlocus(Gm); axis([-15 10 -15 15]); grid on;
    title(sprintf('Root locus of G(s), %s model (gain K)', tag));
    saveas(gcf, fullfile(P.figDir, sprintf('task6_rlocus_%s.png', tag)));
    Ks = logspace(-3, 3, 400);  stableK = false(size(Ks));
    for i = 1:numel(Ks)
        stableK(i) = all(real(pole(feedback(Ks(i)*Gm, 1))) < 0);
    end
    if any(stableK)
        fprintf('P control is stable for K in [%.3g, %.3g] deg/cm\n', ...
                min(Ks(stableK)), max(Ks(stableK)));
    else
        fprintf('P control: NO gain K in [1e-3, 1e3] gives a stable closed loop.\n');
    end
    p1 = pole(feedback(Gm, 1));
    [~, iDom] = max(real(p1));
    fprintf('closed-loop poles with K = 1: %s\n', num2str(p1.', 4));
    fprintf('dominant (right-most) closed-loop pole: %s\n', num2str(p1(iDom), 4));

    %% ---- 6.1 Nyquist ----
    figure; nyquist(Gm); title(sprintf('Nyquist plot of G(s), %s model', tag));
    saveas(gcf, fullfile(P.figDir, sprintf('task6_nyquist_%s.png', tag)));
    figure; nyquist(Gm); axis([-4 2 -3 3]);
    hold on; plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2); hold off;
    title(sprintf('Nyquist plot of G(s), %s model, zoom near -1', tag));
    saveas(gcf, fullfile(P.figDir, sprintf('task6_nyquist_zoom_%s.png', tag)));
    Prhp = sum(real(pG) > 1e-6);  Zrhp = sum(real(p1) > 1e-6);
    fprintf(['Nyquist: open-loop RHP poles P = %d (poles at s = 0 are indented around), ' ...
             'closed-loop RHP poles Z = %d -> N = Z - P = %d clockwise encirclements of -1.\n'], ...
            Prhp, Zrhp, Zrhp - Prhp);

    %% ---- 6.2 Bode and margins ----
    mg = loop_margins(Gm);
    fprintf('Bode (K = 1): phase margin %.1f deg at gain crossover %.3g rad/s\n', mg.PM, mg.wcp);
    if mg.PM > 0
        fprintf('              gain margin %.1f dB at %.3g rad/s\n', mg.GM, mg.wcg);
    else
        fprintf('              no meaningful gain margin: the loop is unstable at K = 1\n');
    end
    w   = logspace(-1, 2, 400);
    H   = squeeze(freqresp(Gm, w));
    mw  = loop_margins(Gm, w);
    fprintf('plant phase: %.0f deg at 0.5 rad/s, %.0f deg at 1 rad/s, %.0f deg at 3 rad/s\n', ...
            interp1(w, mw.phase, 0.5), interp1(w, mw.phase, 1), interp1(w, mw.phase, 3));
    figure;
    subplot(2, 1, 1); semilogx(w, 20*log10(abs(H))); grid on; ylabel('|G| (dB)');
    title(sprintf('Bode plot of G(s), %s model: PM = %.1f deg at %.2f rad/s', tag, mg.PM, mg.wcp));
    subplot(2, 1, 2); semilogx(w, mw.phase, w, -180*ones(size(w)), 'r--'); grid on;
    ylabel('phase (deg)'); xlabel('\omega (rad/s)');
    saveas(gcf, fullfile(P.figDir, sprintf('task6_bode_%s.png', tag)));
    if ~P.isOctave
        figure; margin(Gm); grid on;
        saveas(gcf, fullfile(P.figDir, sprintf('task6_margin_%s.png', tag)));
    end
end
fprintf(['\nInterpretation: the phase of G is at or below -180 deg over the useful band ' ...
         '(double integrator plus actuator/sensor lag), so for ANY\nproportional gain the ' ...
         'phase margin is <= 0: the Nyquist curve passes on the wrong side of -1, and the ' ...
         'root-locus branches leave\nthe poles near s = 0 into the RHP. All three plots ' ...
         'say the same thing: phase must be ADDED near crossover (lead).\n']);

%% ---------------- 6.3 design criteria ----------------
dVdL = 25.3/22^2;                         % V/cm at the Part 1 set point (22 cm)
Pd   = 1/(P.Ts*s + 1);                    % derivative realised over one sample
cases = {
  'unity feedback (C = 1)',        tf(1)
  'Part 1 HW PD (Kp=30, Kd=30)',   dVdL*(30 + 30*s*Pd)
  'Part 1 HW PID (60, 0.5, 50)',   dVdL*(60 + 0.5/s + 50*s*Pd) };
tv = (0:0.01:20)';
fprintf('\n6.3  Existing controllers on the design model (%s); spec: OS < %g %%, Ts(5%%) < %g s\n', ...
        modelName, P.OSmax, P.TsMax);
fprintf('%-30s %8s %8s %9s %9s %s\n', 'controller', 'PM', 'GM(dB)', 'OS(%)', 'Ts(s)', 'meets spec?');
figure; hold on;
leg = {};
for i = 1:size(cases, 1)
    C   = cases{i, 2};
    L   = C*G;
    Tcl = feedback(L, 1);
    mi = loop_margins(L);  pmi = mi.PM;  gmi = min(mi.GM, mi.GMlow);
    if pmi <= 0, gmi = NaN; end           % no meaningful GM for an unstable loop
    if all(real(pole(Tcl)) < 0)
        y  = step(Tcl, tv);
        mt = step_metrics(tv, y, 0, 1, P.band);
        ok = mt.OS < P.OSmax && mt.Ts < P.TsMax;
        plot(tv, y);  leg{end+1} = cases{i, 1}; %#ok<SAGROW>
        fprintf('%-30s %8.1f %8.1f %9.1f %9.2f %s\n', cases{i, 1}, pmi, gmi, ...
                mt.OS, mt.Ts, mat2str(ok));
    else
        fprintf('%-30s %8.1f %8.1f %9s %9s %s\n', cases{i, 1}, pmi, gmi, ...
                'unstable', '-', 'false');
    end
end
plot(tv, (1 + P.band)*ones(size(tv)), 'k:', tv, (1 - P.band)*ones(size(tv)), 'k:');
hold off; grid on; ylim([0 2]);
xlabel('t (s)'); ylabel('y / step size');
legend([leg, {'5 % band'}], 'Location', 'southeast');
title('Unit-step responses of the existing (uncompensated) loops');
saveas(gcf, fullfile(P.figDir, 'task6_existing_steps.png'));
fprintf(['-> the plant cannot meet the specification without dynamic compensation; ' ...
         'see task7_compensator.m.\n']);
