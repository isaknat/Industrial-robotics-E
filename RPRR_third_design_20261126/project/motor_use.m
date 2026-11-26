function r = motor_use(t, Cr, w, wd, tau, m)
% How much of a motor one joint uses over the cycle (Lez_08 MotorSizingNotes p.10, p.21).
%   t        time samples of the cycle [s]
%   Cr       load at the joint without the motor's own rotor [N m, or N for the slider]
%   w, wd    joint speed and acceleration [rad/s, rad/s^2; m/s, m/s^2 for the slider]
%   tau      joint speed / motor speed [-; m/rad for the slider]
%   m        one motor of motor_catalog
%   r.rms    RMS motor torque / rated torque     (must be <= 1)
%   r.peak   peak motor torque / maximum torque  (must be <= 1)
%   r.speed  peak motor speed / maximum speed    (must be <= 1)
% Motor torque: C_m = tau*C_r + J_m*wd/tau.

Cm = tau*Cr + m.Jm*wd/tau;
r.rms   = sqrt(trapz(t, Cm.^2)/(t(end) - t(1)))/m.Cn;
r.peak  = max(abs(Cm))/m.Cmax;
r.speed = max(abs(w))/tau/m.wmax;
end
