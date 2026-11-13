function [best, T] = design_leadlag(G, P)
%DESIGN_LEADLAG  Frequency-response lead(-lag) design by systematic search.
%   [best, T] = design_leadlag(G, P)
%   For every combination of
%     n     number of identical lead sections (1 or 2)
%     wc    gain-crossover frequency (rad/s)
%     a     lead ratio p/z  (max phase per section asin((a-1)/(a+1)))
%     shift lead centre frequency sqrt(z*p) = shift*wc
%     lag   none, or PI-type lag (s+wc/m)/s with m = 4, 6, 10
%   the gain Kc is chosen so that |C(jwc)G(jwc)| = 1 (the crossover really is at
%   wc), then the loop is checked for closed-loop stability, PM, GM, and the
%   prefiltered step response F*T for overshoot and settling time (5 % band).
%   Feasible designs are ranked by how fast a constant input disturbance
%   (e.g. a servo offset) is rejected; among the near-best, the one with the
%   smallest Kc (least noise amplification) is returned.
%   T is a struct array of all feasible designs, best first.
w_grid  = 0.6:0.2:3.0;
a_grid  = [3 4 5 6 8 10];
sh_grid = [0.7 1 1.4];
lag_m   = [0 4 6 10];                 % 0 = no lag
tv      = (0:0.01:30)';
T = struct('n', {}, 'wc', {}, 'a', {}, 'shift', {}, 'Kc', {}, 'z', {}, 'p', {}, ...
           'lagZ', {}, 'lagP', {}, 'PM', {}, 'GM', {}, 'OS', {}, 'Ts', {}, ...
           'tDist', {}, 'eDist', {});
for n = 1:2
for wc = w_grid
for a = a_grid
for sh = sh_grid
for m = lag_m
    d.n = n; d.a = a;
    wm  = sh*wc;  d.z = wm/sqrt(a);  d.p = wm*sqrt(a);
    if m > 0, d.lagZ = wc/m; else, d.lagZ = 0; end
    d.lagP = 0;  d.Kc = 1;
    C1 = leadlag_build(d);
    d.Kc = 1/abs(squeeze(freqresp(C1*G, wc)));
    if d.Kc > P.KcMax, continue; end
    [C, F] = leadlag_build(d);
    L  = C*G;
    Tcl = feedback(L, 1);
    if any(real(pole(Tcl)) >= -1e-6), continue; end
    mg = loop_margins(L);
    pm = mg.PM;  gmdB = min(mg.GM, mg.GMlow);
    if pm < P.PMmin || gmdB < P.GMmin, continue; end
    y  = step(F*Tcl, tv);
    mt = step_metrics(tv, y, 0, 1, P.band);
    if mt.OS >= P.OSmax || mt.Ts >= P.TsMax, continue; end
    yd = step(feedback(G, C), tv);              % 1 deg input disturbance
    pk = max(abs(yd));
    k  = find(abs(yd) > 0.1*pk, 1, 'last');
    r = struct('n', n, 'wc', wc, 'a', a, 'shift', sh, 'Kc', d.Kc, ...
               'z', d.z, 'p', d.p, 'lagZ', d.lagZ, 'lagP', d.lagP, ...
               'PM', pm, 'GM', gmdB, 'OS', mt.OS, 'Ts', mt.Ts, ...
               'tDist', tv(min(k + 1, numel(tv))), 'eDist', abs(yd(end)));
    T(end+1) = r; %#ok<AGROW>
end, end, end, end, end
if isempty(T)
    error('design_leadlag:none', ['No design met the specifications. Relax ' ...
          'P.PMmin / P.KcMax, or extend w_grid in design_leadlag.m.']);
end
score = [T.tDist];
[~, order] = sort(score);
T = T(order);
near = find([T.tDist] <= 1.2*T(1).tDist);
[~, j] = min([T(near).Kc]);
best = T(near(j));
T = [best, T(setdiff(1:numel(T), near(j)))];
end
