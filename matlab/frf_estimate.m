function [w, Gest, coh] = frf_estimate(u, y, c, Ts, fband, nBins)
%FRF_ESTIMATE  Non-parametric frequency response u -> y from a chirp test.
%   [w, Gest] = frf_estimate(u, y, c, Ts, fband, nBins)
%   u, y  : input (deg) and output (cm), means removed
%   c     : the chirp that was injected (used as instrument, so the estimate
%           is not biased by the feedback loop that keeps the ball on the beam)
%   fband : [fmin fmax] in Hz, nBins log-spaced bands are averaged
%   w     : band centre frequencies in rad/s, Gest complex response (cm/deg)
%   coh   : fraction of input power explained by the chirp in each band (0..1);
%           low values mean the estimate in that band is unreliable.
u = u(:); y = y(:); c = c(:);
N  = numel(u);
win = 0.5 - 0.5*cos(2*pi*(0:N-1)'/(N-1));          % Hann window
U = fft(u .* win);  Y = fft(y .* win);  C = fft(c .* win);
f = (0:N-1)' / (N*Ts);
edges = logspace(log10(fband(1)), log10(fband(2)), nBins + 1);
w = nan(nBins, 1);  Gest = nan(nBins, 1);  coh = nan(nBins, 1);
for k = 1:nBins
    b = f >= edges(k) & f < edges(k+1);
    if nnz(b) < 2, continue; end
    Syc = sum(Y(b) .* conj(C(b)));
    Suc = sum(U(b) .* conj(C(b)));
    Gest(k) = Syc / Suc;
    coh(k)  = abs(Suc)^2 / (sum(abs(U(b)).^2) * sum(abs(C(b)).^2));
    w(k)    = 2*pi*sqrt(edges(k) * edges(k+1));
end
ok = ~isnan(Gest);
w = w(ok); Gest = Gest(ok); coh = coh(ok);
end
