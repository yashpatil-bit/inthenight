%% calibrate_sensor.m  -  check / refit the IR distance model of Chirp_Signal_IR
% The instructor's code converts voltage to distance with  L = 35.7 * V^-1.15.
% That is a generic Sharp GP2Y0A21 curve. Measure your own sensor with the ball:
% hold it at known distances from the sensor face, read "mean_V" from
% arduino/sensor_calibration, and fill in the table. A power law is a straight
% line in log-log, so it is fitted with polyfit. If the refit is clearly better,
% put A_CAL/B_CAL into the chirp sketch (35.7, -1.15) AND the controller sketch.
config_part2;

% ---------- measured table: distance (cm)  mean voltage (V) ----------
cal = [ 10   NaN
        12   NaN
        14   NaN
        16   NaN
        18   NaN
        20   NaN
        22   1.280      % Part 1 set point (already measured)
        25   NaN
        28   NaN
        31   NaN
        34   NaN
        37   NaN
        40   NaN ];
% ---------------------------------------------------------------------
cal = cal(~isnan(cal(:, 2)), :);
if size(cal, 1) < 3
    error('calibrate_sensor:data', 'Enter at least 3 measured voltages in the table.');
end
L = cal(:, 1);  V = cal(:, 2);
c = polyfit(log(V), log(L), 1);
B = c(1);  A = exp(c(2));
Lfit = A * V.^B;  Linst = 35.7 * V.^-1.15;
fprintf('\nfit:        L = %.2f * V^%.3f   RMS error %.2f cm\n', A, B, sqrt(mean((Lfit - L).^2)));
fprintf('instructor: L = 35.70 * V^-1.150  RMS error %.2f cm\n', sqrt(mean((Linst - L).^2)));
fprintf('\nArduino (both sketches):\n  35.7 -> %.2f,  -1.15 -> %.3f\n', A, B);

Vp = linspace(0.4, 3.0, 200);
figure;
subplot(1, 2, 1);
plot(V, L, 'o', Vp, A*Vp.^B, '-', Vp, 35.7*Vp.^-1.15, '--');
grid on; xlabel('sensor voltage V (V)'); ylabel('distance L (cm)'); ylim([0 50]);
legend('measured', 'fit', 'instructor model', 'Location', 'northeast');
title('IR sensor calibration');
subplot(1, 2, 2);
plot(L, Lfit - L, 'o-', L, Linst - L, 's--'); grid on;
xlabel('distance L (cm)'); ylabel('error (cm)'); legend('fit', 'instructor model');
title('Distance error');
saveas(gcf, fullfile(P.figDir, 'sensor_calibration.png'));
