function [Gs, prm] = fit_structured(w, Gest, coh)
%FIT_STRUCTURED  Physically structured plant fitted to frequency-response data.
%   [Gs, prm] = fit_structured(w, Gest, coh)
%   Model:  G(s) = K * exp(-Td*s) / ( s^2 * (tau*s + 1) )
%     K   : ball/linkage gain, cm/(deg s^2)   (Part 1 theory: 2.88)
%     tau : servo + sensor-filter lag, s
%     Td  : pure delay (sampling, sensor update, PWM), s
%   The double integrator comes from physics (Part 1 derivation), the two
%   remaining numbers lump the actuator/sensor dynamics. Fitted by minimising
%   the coherence-weighted complex log error with fminsearch.
%   Gs is rational (delay replaced by a 2nd-order Pade approximation) so that
%   rlocus/nyquist/margin all work. prm = [K tau Td].
if nargin < 3 || isempty(coh), coh = ones(size(w)); end
w = w(:); Gest = Gest(:); wt = max(coh(:), 0.05);
model = @(q, w) exp(q(1)) * exp(-1j*w*exp(q(3))) ./ ((1j*w).^2 .* (1 + 1j*w*exp(q(2))));
cost  = @(q) sum(wt .* abs(log(model(q, w) ./ Gest)).^2);
best = []; bestJ = Inf;
for tau0 = [0.03 0.1 0.3]                       % a few starts, keep the best
    for Td0 = [0.02 0.08 0.2]
        K0 = median(abs(Gest) .* w.^2);
        q  = fminsearch(cost, log([K0 tau0 Td0]), optimset('MaxFunEvals', 4000, ...
                        'MaxIter', 4000, 'TolX', 1e-8, 'TolFun', 1e-10, 'Display', 'off'));
        if cost(q) < bestJ, bestJ = cost(q); best = q; end
    end
end
prm = exp(best);
s = tf('s');
Td = prm(3);                                    % 2nd-order Pade of exp(-Td s)
pd = (1 - Td*s/2 + (Td*s)^2/12) / (1 + Td*s/2 + (Td*s)^2/12);
Gs = prm(1) * pd / (s^2 * (prm(2)*s + 1));
end
