function L = robot_params()
% Dimensions of the R-P-R-R robot (Project PDF p.2). Every script reads them
% from here, so a change in one place updates all steps.
%
% Current numbers: the best design of the coarse search in step 3b, chosen
% on 2026-10-24 as the provisional working design (kinematics only; step 5
% torques, the CAD and the real size of the q4 joint may still change them).
% Step 2's example was h 200, alpha 10 deg, L3 250, L4 100.

L.h     = 225;          % [mm]  shoulder height: top of the column, where the rail starts (on the axis)
L.alpha = deg2rad(0);   % [rad] rail angle above horizontal (PDF p.2: alpha); negative = rail tilted down
L.L3    = 300;          % [mm]  q3 joint -> q4 joint
L.L4    = 50;           % [mm]  q4 joint -> pen tip (TCP); 50 = lower edge of the step 3b grid
L.sol   = -1;           % inverse kinematics solution: -1 = q3 joint nearer the axis, +1 = farther out

% Third edition planning bounds. At small slider extensions, admissible shoulder
% angles depend on q2 and q4. See Fusion/v3/joint_grid.json for checked poses.
L.qmin = [-pi;   0; deg2rad(-42); deg2rad(-131)];
L.qmax = [ pi; 141; deg2rad( 45); deg2rad( 131)];

% Joint speed and acceleration limits used to time the trajectory (step 4,
% requirement 2: "considering the motor limits").
% DECIDED 2026-11-21: 3 rad/s and 30 rad/s^2 (slider: 300 mm/s, 3000 mm/s^2).
% With the motors and reductions of step 5 the joints could give at least
% 6.3 rad/s and about 80 rad/s^2 (slider 500 mm/s); the rest is margin for
% the estimated masses and for what the model leaves out. To be checked
% again in step 5 after any change (10/22: acceleration = torque / inertia;
% 11/09: choose, size the motor, check, repeat).
L.vmax  = [3; 300; 3; 3];        % [rad/s; mm/s; rad/s; rad/s]
L.amax  = [30; 3000; 30; 30];    % [rad/s^2; mm/s^2; rad/s^2; rad/s^2]

% Characteristic length used to make the score (1/condition number) compare
% joints of different nature (slider in mm, others in rad) and the pen angle
% with the pen position (Lez_04 p.24: normalize before comparing).   PLACEHOLDER
L.Lc    = 250;          % [mm]
end
