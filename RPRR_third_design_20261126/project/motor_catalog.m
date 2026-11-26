function M = motor_catalog()
% Mitsubishi HG-KR and HG-MR brushless motors up to 200 W (Project PDF p.2).
% Source: Mitsubishi "Servo Motor Instruction Manual (Vol. 3)", chapter 6
% HG-MR/HG-KR series, 6.3.1 "Standard specifications list" (p. 6-4), standard
% type without electromagnetic brake. Read on 2026-11-21 from
% https://www.manualslib.com/manual/1309045/Mitsubishi-Electric-Hg-Mr.html?page=107
%   Cn    rated torque [N m]            Cmax  maximum torque [N m]
%   wn    rated speed [rad/s]           wmax  maximum speed [rad/s]
%   Jm    rotor moment of inertia [kg m^2]      mass [kg]
% All six: rated speed 3000 r/min, maximum speed 6000 r/min.

name = ["HG-MR053", "HG-MR13", "HG-MR23", "HG-KR053", "HG-KR13", "HG-KR23"];
P    = [50, 100, 200, 50, 100, 200];                               % [W]
Cn   = [0.16, 0.32, 0.64, 0.16, 0.32, 0.64];
Cmax = [0.48, 0.95, 1.9,  0.56, 1.1,  2.2];
Jm   = [0.0162, 0.0300, 0.0865, 0.0450, 0.0777, 0.221]*1e-4;
mass = [0.34, 0.54, 0.91, 0.34, 0.54, 0.91];
M = struct('name', num2cell(name), 'P', num2cell(P), 'Cn', num2cell(Cn), 'Cmax', num2cell(Cmax), ...
           'wn', 3000*2*pi/60, 'wmax', 6000*2*pi/60, 'Jm', num2cell(Jm), 'mass', num2cell(mass));
end
