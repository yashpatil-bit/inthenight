function m = step_metrics(t, y, y0, y1, band)
%STEP_METRICS  Overshoot, settling, rise time and final error of a step.
%   m = step_metrics(t, y, y0, y1, band) for a step from y0 to y1 that
%   starts at t(1). band is the settling band as a fraction of |y1-y0|
%   (0.05 for 5 %). Works for steps in either direction. Fields:
%     OS   percent overshoot past y1 (0 if none)
%     Ts   time after which y stays inside y1 +- band*|y1-y0| (Inf if never)
%     Tr   10-90 % rise time (NaN if not reached)
%     ess  y1 - mean of the last 10 % of the record
t = t(:) - t(1);
y = y(:);
span = y1 - y0;
dir  = sign(span);
m.OS  = max(0, max((y - y1) * dir)) / abs(span) * 100;
out   = find(abs(y - y1) > band * abs(span));
if isempty(out)
    m.Ts = 0;
elseif out(end) == numel(y)
    m.Ts = Inf;
else
    m.Ts = t(out(end) + 1);
end
p   = (y - y0) / span;
i10 = find(p >= 0.1, 1);
i90 = find(p >= 0.9, 1);
if isempty(i10) || isempty(i90)
    m.Tr = NaN;
else
    m.Tr = t(i90) - t(i10);
end
nEnd  = max(1, round(0.1 * numel(y)));
m.ess = y1 - mean(y(end-nEnd+1:end));
end
