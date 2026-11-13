function m = loop_margins(L, w)
%LOOP_MARGINS  Phase and gain margins with a correctly unwrapped phase.
%   m = loop_margins(L) for an open-loop transfer function L(s).
%   The phase is unwrapped from low frequency and anchored to the integrator
%   count (k poles at s = 0 -> phase starts near -90k deg), so a double
%   integrator gives PM < 0 instead of a wrapped value such as 349 deg.
%     m.PM, m.wcp   phase margin (deg) at the (highest) gain crossover
%     m.GM, m.wcg   gain margin (dB) at the first -180 deg crossing ABOVE wcp
%                   (how much the gain may be RAISED); Inf if none
%     m.GMlow       gain-reduction margin (dB) at a -180 deg crossing BELOW wcp
%                   (conditionally stable loops, e.g. lead + integrating lag);
%                   how much the gain may be LOWERED; Inf if none
%     m.w, m.mag, m.phase   the curves used
if nargin < 2, w = logspace(-3, 3, 6000); end
H   = squeeze(freqresp(L, w));  H = H(:).';
mag = abs(H);
ph  = unwrap(angle(H)) * 180/pi;
p   = pole(L);
k   = sum(abs(p) < 1e-6 * max(1, max(abs(p))));
ph  = ph - 360*round((ph(1) + 90*k)/360);     % anchor at -90k deg
m.w = w;  m.mag = mag;  m.phase = ph;

ic = find(diff(sign(mag - 1)) ~= 0);
if isempty(ic)
    m.wcp = NaN;  m.PM = Inf;
else
    i = ic(end);
    f = (0 - log(mag(i))) / (log(mag(i+1)) - log(mag(i)));
    m.wcp = exp(log(w(i)) + f*(log(w(i+1)) - log(w(i))));
    m.PM  = 180 + interp1(log(w), ph, log(m.wcp));
end

q  = (ph + 180)/360;                         % crosses an integer at -180 - 360n
ip = find(floor(q(1:end-1)) ~= floor(q(2:end)));
m.GM = Inf;  m.wcg = NaN;  m.GMlow = Inf;
for i = ip
    wi = w(i);  g = -20*log10(mag(i));
    if ~isnan(m.wcp) && wi > m.wcp
        if isinf(m.GM), m.GM = g; m.wcg = wi; end
    elseif ~isnan(m.wcp)
        m.GMlow = min(m.GMlow, -g);
    end
end
end
