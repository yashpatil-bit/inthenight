function [C, F, Clead] = leadlag_build(d)
%LEADLAG_BUILD  Continuous compensator and reference prefilter from a design.
%   d.Kc, d.n, d.z, d.p          C_lead = Kc*((s+z)/(s+p))^n
%   d.lagZ, d.lagP (d.lagZ = 0 -> no lag)   Lag = (s+lagZ)/(s+lagP)
%   C = C_lead*Lag,  F = (z/(s+z))^n  (cancels the lead zeros for set-point steps)
s = tf('s');
Clead = d.Kc * ((s + d.z)/(s + d.p))^d.n;
if d.lagZ > 0
    C = Clead * (s + d.lagZ)/(s + d.lagP);
else
    C = Clead;
end
F = (d.z/(s + d.z))^d.n;
end
