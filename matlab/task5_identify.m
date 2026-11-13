%% task5_identify.m  -  Task 5: system characterisation from the chirp test
% 1. loads the chirp log (u = servo deviation in deg, y = ball distance in cm)
% 2. non-parametric frequency response (chirp used as instrument)
% 3. n4sid models of several orders -> continuous transfer functions (d2c).
%    The data come from a closed loop (the PID keeps the ball on the beam), so
%    sensor noise is fed back into u and plain n4sid is biased. The two-stage
%    method removes this: stage 1 keeps only the part of u that is caused by the
%    chirp (least-squares FIR fit u ~ chirp), stage 2 runs n4sid from that
%    noise-free input to y. In MATLAB the direct n4sid with SSARX weighting
%    (designed for closed-loop data) is tried as well; the best model wins.
% 4. validation: each model is simulated inside the same PD + chirp loop that was
%    used in the experiment and compared with the measured y (an open-loop
%    simulation of a double integrator drifts away, so this is the fair test)
% 5. a physically structured fit K*exp(-Td s)/(s^2(tau s+1)) to the same data
% 6. Bode plot of both models vs. the data and vs. the Part 1 model
% Output: results/plant_model.mat, figures/task5_*.png
config_part2;

%% ---------------- load and pre-process ----------------
D   = read_log(fullfile(P.dataDir, P.chirpFile));
idx = D.t >= P.tChirp(1) - 1 & D.t <= P.tChirp(2) + 2;
% u = servo deviation from the level angle, in the "plant" sign convention
t = D.t(idx);  y = D.dist_cm(idx);
u = P.direction * (D.servo_deg(idx) - P.centerAngle);
if isfield(D, 'chirp_deg'), c = P.direction * D.chirp_deg(idx); else, c = u; end
bad = y < 0;                                   % -1 = invalid IR reading
if any(bad)
    y(bad) = interp1(t(~bad), y(~bad), t(bad), 'linear', 'extrap');
    fprintf('%d invalid distance samples interpolated\n', nnz(bad));
end
Ts = median(diff(t));
fprintf('sample time in the log: %.4f s\n', Ts);
u0 = mean(u);  y0 = mean(y);
ud = u - u0;   yd = y - y0;
fprintf('operating point: u0 = %.2f deg (servo offset needed to hold the ball), y0 = %.2f cm\n', u0, y0);

figure;
subplot(3, 1, 1); plot(D.t, D.dist_cm); grid on; ylabel('y (cm)');
title('Chirp experiment (raw log)');
subplot(3, 1, 2); plot(D.t, D.servo_deg); grid on; ylabel('servo (deg)');
subplot(3, 1, 3); plot(D.t, D.chirp_deg); grid on; ylabel('chirp (deg)'); xlabel('t (s)');
saveas(gcf, fullfile(P.figDir, 'task5_chirp_data.png'));

%% ---------------- non-parametric estimate ----------------
[wE, GE, coh] = frf_estimate(ud, yd, c, Ts, P.chirpBand, 20);
Kest = median(abs(GE(wE < 2*pi*0.5)) .* wE(wE < 2*pi*0.5).^2);
fprintf('low-frequency gain |G(jw)|*w^2 = %.2f cm/(deg s^2)   (Part 1 model: %.2f)\n', ...
        Kest, P.Ktheory);

%% ---------------- validation data ----------------
haveVal = ~isempty(P.chirpValFile) && exist(fullfile(P.dataDir, P.chirpValFile), 'file');
if haveVal
    V  = read_log(fullfile(P.dataDir, P.chirpValFile));
    iv = V.t >= P.tChirp(1) - 1 & V.t <= P.tChirp(2) + 2;
    vt = V.t(iv);  vy = V.dist_cm(iv);
    if isfield(V, 'chirp_deg'), vc = P.direction * V.chirp_deg(iv);
    else, vc = P.direction * (V.servo_deg(iv) - P.centerAngle); end
    ok = vy >= 0;  vy(~ok) = interp1(vt(ok), vy(ok), vt(~ok), 'linear', 'extrap');
    vy = vy - mean(vy);
else
    vt = t;  vc = c;  vy = yd;
    fprintf('(no validation file - validating on the estimation run)\n');
end
s   = tf('s');
if P.idHold
    Cpd = P.idKp + P.idKi/s + P.idKd*s/(P.idTauD*s + 1);   % ball-holding PID
else
    Cpd = tf(0);                                           % open-loop experiment
end

%% ---------------- stage 1: part of u explained by the chirp ----------------
M   = round(8/Ts);                           % FIR length: 8 s of chirp history
Phi = zeros(numel(c), M);
for j = 1:M
    Phi(j:end, j) = c(1:end-j+1);
end
theta = (Phi'*Phi + 1e-6*eye(M)) \ (Phi'*ud);
uhat  = Phi*theta;
fprintf('stage 1: chirp explains %.0f %% of the input variance\n', ...
        100*(1 - var(ud - uhat)/var(ud)));

%% ---------------- stage 2: n4sid for several orders ----------------
R    = max(1, round(P.TsId/Ts));           % block-average to P.TsId
nb   = floor(numel(yd)/R);
blk  = @(x) mean(reshape(x(1:nb*R), R, nb), 1).';
TsN  = R*Ts;  yN = blk(yd);  uhN = blk(uhat);  uN = blk(ud);
fprintf('n4sid on %d samples at Ts = %.2f s\n', nb, TsN);
cands = {};  names = {};
for n = P.orders
    data = iddata(yN, uhN, TsN);
    if P.isOctave
        sysd = n4sid(data, n);
        sysd = ss(sysd.a, sysd.b(:, 1), sysd.c, 0, TsN);
    else
        sysd = ss(n4sid(data, n, 'Feedthrough', false));   % measured channel only
    end
    cands{end+1} = sysd;  names{end+1} = sprintf('two-stage n4sid, order %d', n); %#ok<SAGROW>
    if ~P.isOctave
        opt  = n4sidOptions('N4Weight', 'SSARX');            % for closed-loop data
        sysd = ss(n4sid(iddata(yN, uN, TsN), n, 'Feedthrough', false, opt));
        cands{end+1} = sysd;  names{end+1} = sprintf('direct n4sid (SSARX), order %d', n); %#ok<SAGROW>
    end
end
models = {};  fits = [];  frfErr = [];  orders = [];
for i = 1:numel(cands)
    sysd = cands{i};
    n    = size(sysd.a, 1);
    try
        Gc = d2c(sysd, 'zoh');
    catch
        Gc = d2c(sysd, 'tustin');                        % negative real pole
    end
    Gc = minreal(tf(Gc), 1e-4);
    % closed-loop simulation in the experiment's loop: chirp -> y
    ysim = lsim(feedback(Gc, Cpd), vc, vt - vt(1));
    fit  = 100*(1 - norm(vy - ysim)/norm(vy - mean(vy)));
    Gm   = squeeze(freqresp(Gc, wE));
    err  = mean(abs(log(Gm(:) ./ GE(:))));
    if ~isfinite(fit), fit = -Inf; end
    models{end+1} = Gc;  fits(end+1) = fit;  frfErr(end+1) = err;  orders(end+1) = n; %#ok<SAGROW>
    fprintf('%-34s: closed-loop simulation fit %6.1f %%, frequency-response error %.3f\n', ...
            names{i}, fit, err);
end
if isempty(P.forceOrder)
    [~, k] = max(fits);
else
    ok = find(orders == P.forceOrder);
    [~, j] = max(fits(ok));  k = ok(j);
end
Gn4 = models{k};  nBest = orders(k);  fitN4 = fits(k);
fprintf('\nbest n4sid model: %s (cm/deg):\n', names{k});
Gn4
fprintf('poles: %s\n', num2str(pole(Gn4).', 4));
fprintf('zeros: %s\n', num2str(zero(Gn4).', 4));

%% ---------------- physically structured model ----------------
% n4sid only "sees" the excited band (chirp from F0). Below it the data say
% nothing, so n4sid may put the two integrator poles at small negative values.
% Physics (Part 1) says the plant IS a double integrator; this fit keeps that.
[Gst, prm] = fit_structured(wE, GE, coh);
ysimS = lsim(feedback(Gst, Cpd), vc, vt - vt(1));
fitSt = 100*(1 - norm(vy - ysimS)/norm(vy - mean(vy)));
errSt = mean(abs(log(squeeze(freqresp(Gst, wE)) ./ GE(:))));
fprintf(['\nstructured model K*exp(-Td s)/(s^2 (tau s+1)):  K = %.3f cm/(deg s^2), ' ...
         'tau = %.3f s, Td = %.3f s\n'], prm(1), prm(2), prm(3));
fprintf('structured model: closed-loop simulation fit %6.1f %%, frequency-response error %.3f\n', ...
        fitSt, errSt);
fprintf('Part 1 theoretical gain 2.88 cm/(deg s^2) -> identified/theory = %.2f\n', prm(1)/P.Ktheory);

% model used for Tasks 6-8: P.designModel = 'auto' | 'structured' | 'n4sid'
if ~isfield(P, 'designModel'), P.designModel = 'auto'; end
useStruct = strcmp(P.designModel, 'structured') || ...
            (strcmp(P.designModel, 'auto') && fitSt >= fitN4 - 5);
if useStruct
    G = Gst;  modelName = 'structured fit';
else
    G = Gn4;  modelName = sprintf('n4sid order %d', nBest);
end
fprintf('\nmodel used for design: %s\n', modelName);

%% ---------------- figures ----------------
ysimN = lsim(feedback(Gn4, Cpd), vc, vt - vt(1));
figure;
plot(vt, vy, vt, ysimN, '--', vt, ysimS, '-.'); grid on;
xlabel('t (s)'); ylabel('y - mean (cm)');
legend('measured', sprintf('n4sid order %d, fit %.0f %%', nBest, fitN4), ...
       sprintf('structured model, fit %.0f %%', fitSt));
title('Validation: models simulated in the PID + chirp loop');
saveas(gcf, fullfile(P.figDir, 'task5_validation.png'));

w   = logspace(log10(2*pi*P.chirpBand(1)/3), log10(2*pi*P.chirpBand(2)*3), 300);
Gth = P.Ktheory/s^2;
Hn  = squeeze(freqresp(Gn4, w));  Hs = squeeze(freqresp(Gst, w));
Ht  = squeeze(freqresp(Gth, w));
ph  = @(H) unwrap(angle(H(:))) * 180/pi;
near180 = @(p) p - 360*round((p(1) + 180)/360);
pn = near180(ph(Hn));  ps = near180(ph(Hs));
pe = ph(GE);  pe = pe - 360*round((pe(1) - interp1(w, ps, wE(1)))/360);
figure;
subplot(2, 1, 1);
semilogx(w, 20*log10(abs(Hn)), w, 20*log10(abs(Hs)), '-.', w, 20*log10(abs(Ht)), ':', ...
         wE, 20*log10(abs(GE)), 'o');
grid on; ylabel('magnitude (dB, cm/deg)');
legend(sprintf('n4sid (order %d)', nBest), 'structured fit', 'Part 1 model 2.88/s^2', 'chirp data');
title('Task 5: Bode plot of the identified plant');
subplot(2, 1, 2);
semilogx(w, pn, w, ps, '-.', w, -180*ones(size(w)), ':', wE, pe, 'o');
grid on; ylabel('phase (deg)'); xlabel('\omega (rad/s)');
saveas(gcf, fullfile(P.figDir, 'task5_bode.png'));

figure; pzmap(Gn4); title(sprintf('Poles and zeros of the n4sid model (order %d)', nBest));
saveas(gcf, fullfile(P.figDir, 'task5_pzmap.png'));

Gd = c2d(G, Ts, 'zoh');
save(fullfile(P.resDir, 'plant_model.mat'), 'G', 'Gd', 'Gn4', 'Gst', 'prm', 'modelName', ...
     'nBest', 'Kest', 'u0', 'y0', 'fits', 'frfErr', 'fitN4', 'fitSt');
fprintf('saved results/plant_model.mat\n');
