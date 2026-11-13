%% task7_compensator.m  -  Task 7: lead or lag? design and verification
% A. why LEAD (and not lag) is required - from the Task 6 phase curve
% B. systematic frequency-response design (design_leadlag.m): lead for phase,
%    plus an integrating lag for steady-state accuracy, plus a reference prefilter
% C. textbook lead numbers for the chosen design (phase needed, alpha, z, p, Kc)
% D. verification: Bode before/after, step responses, disturbance rejection,
%    servo effort, compensated root locus and Nyquist
% E. discrete-time check at the Arduino rate and the block to paste in the sketch
% Output: results/compensator.mat, figures/task7_*.png
config_part2;
load(fullfile(P.resDir, 'plant_model.mat'), 'G', 'Gn4', 'modelName');
s  = tf('s');
tv = (0:0.01:15)';

%% ---------------- A. lead or lag? ----------------
m0 = loop_margins(G);
fprintf('A. Uncompensated plant (%s): PM = %.1f deg at %.2f rad/s\n', modelName, m0.PM, m0.wcp);
fprintf(['   G has phase <= -180 deg at all frequencies. A lag compensator only adds ' ...
         'NEGATIVE phase, so it cannot\n   stabilise the loop at any gain. A lead ' ...
         'compensator adds POSITIVE phase near crossover -> LEAD is required.\n' ...
         '   A lag/integral part is then added for the steady state: on hardware a ' ...
         'servo-centre error or friction acts as a\n   constant input disturbance d, ' ...
         'leaving an error d/C(0) that only a large low-frequency gain removes.\n' ...
         '   => compensator type: LEAD-LAG (lead for stability/transient, lag for ' ...
         'steady state).\n\n']);

%% ---------------- B. design ----------------
[best, T] = design_leadlag(G, P);
fprintf('B. %d designs meet OS < %g %%, Ts < %g s, PM >= %g, GM >= %g dB, Kc <= %g\n', ...
        numel(T), P.OSmax, P.TsMax, P.PMmin, P.GMmin, P.KcMax);
fprintf('   top designs (first = chosen):\n');
fprintf('   %2s %5s %4s %5s %7s %7s %7s %6s %6s %6s %6s %6s %7s\n', 'n', 'wc', 'a', 'shift', ...
        'Kc', 'z', 'p', 'lagZ', 'PM', 'GM', 'OS%', 'Ts', 'tDist');
for i = 1:min(6, numel(T))
    r = T(i);
    fprintf('   %2d %5.2f %4g %5.1f %7.3f %7.3f %7.3f %6.3f %6.1f %6.1f %6.2f %6.2f %7.2f\n', ...
            r.n, r.wc, r.a, r.shift, r.Kc, r.z, r.p, r.lagZ, r.PM, r.GM, r.OS, r.Ts, r.tDist);
end
d = best;
[C, F, Clead] = leadlag_build(d);

%% ---------------- C. textbook view of the chosen design ----------------
phG   = interp1(log(m0.w), m0.phase, log(d.wc));
phLag = 0;
if d.lagZ > 0, phLag = atan(d.wc/d.lagZ)*180/pi - 90; end
phiM  = asin((d.a - 1)/(d.a + 1))*180/pi;
fprintf('\nC. Chosen design, read as a classical lead(-lag) design at crossover wc = %.2f rad/s:\n', d.wc);
fprintf('   plant phase at wc            %7.1f deg\n', phG);
fprintf('   integrating lag adds          %7.1f deg\n', phLag);
fprintf('   lead sections: %d x (s+%.3f)/(s+%.3f), ratio a = p/z = %g\n', d.n, d.z, d.p, d.a);
fprintf('   max phase per section        %7.1f deg at %.2f rad/s (= %.1f x wc)\n', ...
        phiM, sqrt(d.z*d.p), d.shift);
phLead = d.n*(atan(d.wc/d.z) - atan(d.wc/d.p))*180/pi;
fprintf('   lead phase actually at wc     %7.1f deg\n', phLead);
fprintf('   => PM = 180 + %.1f + %.1f + %.1f = %.1f deg\n', phG, phLag, phLead, ...
        180 + phG + phLag + phLead);
fprintf('   gain Kc = %.3f deg/cm puts |C G| = 1 at wc\n', d.Kc);
fprintf('   prefilter F(s) = (%.3f/(s+%.3f))^%d cancels the lead zeros for set-point steps\n', ...
        d.z, d.z, d.n);
fprintf('C(s) ='); C

%% ---------------- D. verification ----------------
mL = loop_margins(Clead*G);  mC = loop_margins(C*G);
fprintf('D. margins: lead only PM %.1f deg, GM %.1f dB | lead+lag PM %.1f deg, GM %.1f dB (gain-reduction %.1f dB)\n', ...
        mL.PM, mL.GM, mC.PM, mC.GM, mC.GMlow);
w = logspace(-2, 2, 600);
figure;
subplot(2, 1, 1);
semilogx(w, 20*log10(abs(squeeze(freqresp(G, w)))), ':', ...
         w, 20*log10(abs(squeeze(freqresp(Clead*G, w)))), '--', ...
         w, 20*log10(abs(squeeze(freqresp(C*G, w)))), '-');
grid on; ylabel('magnitude (dB)'); ylim([-60 60]);
legend('plant G', 'lead \cdot G', 'lead-lag \cdot G', 'Location', 'southwest');
title(sprintf('Task 7: loop Bode plots (PM: %.0f -> %.0f deg)', m0.PM, mC.PM));
subplot(2, 1, 2);
mGw = loop_margins(G, w);  mLw = loop_margins(Clead*G, w);  mCw = loop_margins(C*G, w);
semilogx(w, mGw.phase, ':', w, mLw.phase, '--', w, mCw.phase, '-', w, -180*ones(size(w)), 'k');
grid on; ylabel('phase (deg)'); xlabel('\omega (rad/s)'); ylim([-360 0]);
saveas(gcf, fullfile(P.figDir, 'task7_bode.png'));

Tcl = feedback(C*G, 1);  TclL = feedback(Clead*G, 1);
yPF = step(F*Tcl, tv);  yNo = step(Tcl, tv);  yL = step(F*TclL, tv);
mPF = step_metrics(tv, yPF, 0, 1, P.band);
mNo = step_metrics(tv, yNo, 0, 1, P.band);
mLd = step_metrics(tv, yL, 0, 1, P.band);
fprintf('   step, lead-lag + prefilter : OS %5.2f %%, Ts %5.2f s, Tr %5.2f s, ess %.4f  <-- design\n', ...
        mPF.OS, mPF.Ts, mPF.Tr, mPF.ess);
fprintf('   step, lead-lag, no prefilter: OS %5.2f %%, Ts %5.2f s\n', mNo.OS, mNo.Ts);
fprintf('   step, lead only + prefilter : OS %5.2f %%, Ts %5.2f s\n', mLd.OS, mLd.Ts);
figure;
plot(tv, yPF, '-', tv, yNo, '--', tv, yL, '-.', ...
     tv, (1 + P.band)*ones(size(tv)), 'k:', tv, (1 - P.band)*ones(size(tv)), 'k:');
grid on; xlabel('t (s)'); ylabel('y / step size');
legend(sprintf('lead-lag + prefilter (OS %.1f%%, Ts %.2f s)', mPF.OS, mPF.Ts), ...
       sprintf('lead-lag, no prefilter (OS %.0f%%)', mNo.OS), ...
       sprintf('lead only + prefilter (OS %.1f%%)', mLd.OS), '5 % band', ...
       'Location', 'southeast');
title('Task 7: closed-loop step responses (identified model)');
saveas(gcf, fullfile(P.figDir, 'task7_step.png'));

td = (0:0.01:30)';
yd  = step(feedback(G, C), td);      ydL = step(feedback(G, Clead), td);
fprintf('   1 deg input offset: lead only leaves %.2f cm, lead-lag %.3f cm after 30 s\n', ...
        ydL(end), yd(end));
figure; plot(td, ydL, '--', td, yd, '-'); grid on;
xlabel('t (s)'); ylabel('ball position error (cm)');
legend('lead only', 'lead + integrating lag');
title('Response to a 1 deg constant servo offset (input disturbance)');
saveas(gcf, fullfile(P.figDir, 'task7_disturbance.png'));

uStep = step(F*feedback(C, G), tv) * 10;        % servo effort for a 10 cm step
fprintf('   servo effort for a 10 cm set-point step: peak %.1f deg (limit +-30)\n', ...
        max(abs(uStep)));
figure; plot(tv, uStep); grid on; xlabel('t (s)'); ylabel('u (deg)');
title('Servo command for a 10 cm set-point step');
saveas(gcf, fullfile(P.figDir, 'task7_effort.png'));

figure; rlocus(C*G); axis([-8 2 -6 6]); hold on;
pc = pole(Tcl);  plot(real(pc), imag(pc), 'rs', 'MarkerSize', 8, 'LineWidth', 2); hold off;
title('Root locus of the compensated loop (squares: chosen gain)');
saveas(gcf, fullfile(P.figDir, 'task7_rlocus.png'));
figure; nyquist(C*G); axis([-3 1 -2 2]); hold on; plot(-1, 0, 'r+', 'MarkerSize', 12); hold off;
title('Nyquist plot of the compensated loop');
saveas(gcf, fullfile(P.figDir, 'task7_nyquist.png'));
fprintf('   closed-loop poles: %s\n', num2str(pc.', 4));

% cross-check: the same compensator on the n4sid "real transfer function"
mN = loop_margins(C*Gn4);
TclN = feedback(C*Gn4, 1);
if all(real(pole(TclN)) < 0)
    yN = step(F*TclN, tv);  mtN = step_metrics(tv, yN, 0, 1, P.band);
    fprintf(['   cross-check on the n4sid model: PM %.1f deg, GM %.1f dB, OS %.2f %%, ' ...
             'Ts %.2f s, stable\n'], mN.PM, mN.GM, mtN.OS, mtN.Ts);
    figure; plot(tv, yPF, '-', tv, yN, '--', tv, (1 + P.band)*ones(size(tv)), 'k:', ...
                 tv, (1 - P.band)*ones(size(tv)), 'k:'); grid on;
    xlabel('t (s)'); ylabel('y / step size');
    legend(sprintf('design model (%s)', modelName), 'n4sid model', '5 % band', ...
           'Location', 'southeast');
    title('Compensated step response on both identified models');
    saveas(gcf, fullfile(P.figDir, 'task7_step_both_models.png'));
else
    fprintf('   cross-check on the n4sid model: UNSTABLE - redesign with P.designModel = ''n4sid''\n');
end

%% ---------------- E. discrete-time check and Arduino block ----------------
Cd = c2d(C, P.Ts, 'tustin');  Fd = c2d(F, P.Ts, 'tustin');  Gd = c2d(G, P.Ts, 'zoh');
Tdz = feedback(Cd*Gd, 1);
if any(abs(pole(Tdz)) >= 1)
    warning('discrete loop at Ts = %.3f s is UNSTABLE - lower the crossover', P.Ts);
end
kd  = (0:round(tv(end)/P.Ts))';
yDz = step(Fd*Tdz, kd*P.Ts);
mDz = step_metrics(kd*P.Ts, yDz, 0, 1, P.band);
fprintf('E. discrete (Ts = %.3f s, Tustin): OS %.2f %%, Ts %.2f s\n', P.Ts, mDz.OS, mDz.Ts);
figure; plot(tv, yPF, '-', kd*P.Ts, yDz, 'o', 'MarkerSize', 3); grid on;
xlabel('t (s)'); ylabel('y / step size'); legend('continuous', sprintf('discrete, T = %g s', P.Ts));
title('Continuous vs discrete implementation');
saveas(gcf, fullfile(P.figDir, 'task7_discrete.png'));

fprintf('\n---- paste into arduino/leadlag_controller/leadlag_controller.ino ----\n');
fprintf('const float   KC           = %.4ff;   // deg per cm\n', d.Kc);
fprintf('const int     N_LEAD       = %d;\n', d.n);
fprintf('const float   LEAD_Z       = %.4ff;   // rad/s\n', d.z);
fprintf('const float   LEAD_P       = %.4ff;   // rad/s\n', d.p);
fprintf('const bool    USE_LAG      = %s;\n', mat2str(d.lagZ > 0));
fprintf('const float   LAG_Z        = %.4ff;   // rad/s\n', max(d.lagZ, 0.1));
fprintf('const float   LAG_P        = %.4ff;   // rad/s (0 = integrating, PI-type lag)\n', d.lagP);
fprintf('const bool    USE_PREFILTER = true;\n');
fprintf('-----------------------------------------------------------------------\n');
save(fullfile(P.resDir, 'compensator.mat'), 'd', 'C', 'F', 'Clead', 'Cd', 'Fd', 'T');
fprintf('saved results/compensator.mat\n');
