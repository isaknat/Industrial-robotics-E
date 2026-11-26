function [P, n, phi] = wall_point(u, v, prm)
% Flat letter point (u, v) -> point on the wall and pen direction there.
% Same "label on a can" wrapping as step1_surface_E (step 4 checks that both agree).
%   u    flat horizontal coordinate [mm], to the right as seen from the robot
%   v    flat vertical coordinate [mm], measured from the letter's bottom
%   prm  parameters saved by step 1 (R0, A, H, k, E_z0, phi_c)
%   P    [3 x m] points on the wall
%   n    [3 x m] pen direction: unit, perpendicular to the wall, pointing into it
%   phi  [1 x m] angle around the axis

z   = prm.E_z0 + v;
phi = prm.phi_c - u/prm.R0;
w   = 2*pi*prm.k/prm.H;
rho = prm.R0 + prm.A*sin(w*z);
drz = prm.A*w*cos(w*z);
P   = [rho.*cos(phi); rho.*sin(phi); z];
n   = [cos(phi); sin(phi); -drz];
n   = n./vecnorm(n);
end
