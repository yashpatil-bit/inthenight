%% task8_hardware_eval.m  -  Task 8: evaluate the compensator on the hardware
% Needs two logs from arduino/leadlag_controller (see README):
%   P.stepFile : step mode ("s"), set point alternating REF_LOW <-> REF_HIGH
%   P.kickFile : constant set point, ball nudged by hand a few times
% For every set-point step: overshoot, 5 % settling time, rise time, final
% error, measured on a lightly smoothed signal (zero-phase moving average of
% P.smoothSec; the raw IR noise is about as large as the 5 % band). The
% measured response is overlaid on the model prediction from Task 7.
% For every nudge: peak deviation and time to return within +-1 cm.
% Output: figures/task8_*.png, results/hardware_metrics.mat
config_part2;
load(fullfile(P.resDir, 'plant_model.mat'), 'G');
load(fullfile(P.resDir, 'compensator.mat'), 'C', 'F', 'd');
smoothv = @(x, k) conv(x, ones(2*k + 1, 1), 'same') ./ conv(ones(size(x)), ones(2*k + 1, 1), 'same');
ks = max(1, round(P.smoothSec/P.Ts/2));

%% ---------------- set-point steps ----------------
S  = read_log(fullfile(P.dataDir, P.stepFile));
t  = S.t;  y = S.dist_cm;  r = S.ref_cm;  ys = smoothv(y, ks);
edges = find(diff(r) ~= 0) + 1;
fprintf('\nTask 8 - set-point steps (metrics on %.2f s smoothed signal, %g %% band)\n', ...
        P.smoothSec, 100*P.band);
fprintf('%6s %12s %8s %8s %8s %9s\n', 't (s)', 'step (cm)', 'OS (%)', 'Ts (s)', 'Tr (s)', 'ess (cm)');
M = [];
for j = 1:numel(edges)
    i0 = edges(j);
    if j < numel(edges), i1 = edges(j+1) - 1; else, i1 = numel(t); end
    if t(i1) - t(i0) < 8 || t(i0) < 3, continue; end   % incomplete / start-up segment
    m = step_metrics(t(i0:i1), ys(i0:i1), r(i0 - 1), r(i0), P.band);
    M = [M; t(i0), r(i0-1), r(i0), m.OS, m.Ts, m.Tr, m.ess]; %#ok<AGROW>
    fprintf('%6.1f %5.1f->%5.1f %8.2f %8.2f %8.2f %+9.2f\n', t(i0), r(i0-1), r(i0), ...
            m.OS, m.Ts, m.Tr, m.ess);
end
if ~isempty(M)
    fprintf('mean: OS %.2f %% (max %.2f), Ts %.2f s (max %.2f), |ess| %.2f cm\n', ...
            mean(M(:, 4)), max(M(:, 4)), mean(M(:, 5)), max(M(:, 5)), mean(abs(M(:, 7))));
    fprintf('spec OS < %g %% and Ts < %g s met on every step: %s\n', P.OSmax, P.TsMax, ...
            mat2str(all(M(:, 4) < P.OSmax & M(:, 5) < P.TsMax)));
end

% model prediction for the same reference signal (linear, around the first level)
if ~isempty(edges)
    yModel = lsim(F*feedback(C*G, 1), r - r(edges(1) - 1), t) + r(edges(1) - 1);
else
    yModel = r;
end
figure;
subplot(2, 1, 1);
plot(t, y, 'Color', [0.7 0.7 0.7]); hold on;
plot(t, ys, 'b', t, yModel, 'r--', t, r, 'k:'); hold off; grid on;
ylabel('ball distance (cm)');
legend('measured (raw)', 'measured (smoothed)', 'model prediction', 'set point', ...
       'Location', 'best');
title('Task 8: lead-lag compensator on the hardware - set-point steps');
subplot(2, 1, 2); plot(t, S.u_deg); grid on; xlabel('t (s)'); ylabel('servo u (deg)');
saveas(gcf, fullfile(P.figDir, 'task8_steps.png'));

if ~isempty(M)          % zoom on the first complete step
    j  = find(M(:, 1) == M(1, 1));  i0 = find(t >= M(j, 1), 1);
    i1 = min(numel(t), i0 + round(10/P.Ts));
    span = M(j, 3) - M(j, 2);
    figure;
    plot(t(i0:i1) - t(i0), ys(i0:i1), 'b', t(i0:i1) - t(i0), yModel(i0:i1), 'r--'); hold on;
    plot([0 10], M(j, 3) + P.band*abs(span)*[1 1], 'k:', [0 10], M(j, 3) - P.band*abs(span)*[1 1], 'k:');
    hold off; grid on; xlabel('time after step (s)'); ylabel('ball distance (cm)');
    legend('measured (smoothed)', 'model', '5 % band', 'Location', 'best');
    title(sprintf('Step %.0f -> %.0f cm: OS %.1f %%, Ts %.2f s', M(j, 2), M(j, 3), M(j, 4), M(j, 5)));
    saveas(gcf, fullfile(P.figDir, 'task8_step_zoom.png'));
end

%% ---------------- perturbations ----------------
K   = read_log(fullfile(P.dataDir, P.kickFile));
tk  = K.t;  ek = K.dist_cm - K.ref_cm;  eks = smoothv(ek, ks);
noiseStd = std(ek(tk > 2 & tk < 8) - eks(tk > 2 & tk < 8));
fprintf('\nTask 8 - perturbations (ball nudged by hand), sensor noise std %.2f cm\n', noiseStd);
fprintf('%8s %12s %16s\n', 't (s)', 'peak (cm)', 'recovery (s)');
big = find(abs(eks) > 2.5 & tk > 3);  % samples clearly disturbed (skip start-up)
events = [];
if ~isempty(big)
    starts = big([true; diff(big) > round(3/P.Ts)]);
    for i0 = starts.'
        iEnd = min(numel(tk), i0 + round(12/P.Ts));
        [pk, ip] = max(abs(eks(i0:iEnd)));
        ip = i0 + ip - 1;
        out = find(abs(eks(ip:iEnd)) > 1.0, 1, 'last');
        trec = tk(min(iEnd, ip + out)) - tk(i0);
        events = [events; tk(i0), pk, trec]; %#ok<AGROW>
        fprintf('%8.1f %12.2f %16.2f\n', tk(i0), pk, trec);
    end
end
figure;
subplot(2, 1, 1);
plot(tk, K.dist_cm, 'Color', [0.7 0.7 0.7]); hold on;
plot(tk, eks + K.ref_cm, 'b', tk, K.ref_cm, 'k:'); hold off; grid on;
ylabel('ball distance (cm)'); legend('raw', 'smoothed', 'set point');
title('Task 8: response to perturbations (ball nudged by hand)');
subplot(2, 1, 2); plot(tk, K.u_deg); grid on; xlabel('t (s)'); ylabel('servo u (deg)');
saveas(gcf, fullfile(P.figDir, 'task8_perturbation.png'));

save(fullfile(P.resDir, 'hardware_metrics.mat'), 'M', 'events', 'noiseStd');
fprintf('saved results/hardware_metrics.mat\n');
