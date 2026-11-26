function D = robot_masses(m_payload)
% Masses of the R-P-R-R robot for the dynamic model (step 5), in SI units.
%   m_payload  mass carried at the pen tip [kg] (Project: 0.2 to 1 kg); default 1
%
% Fusion reports SI mass, centre of mass and centroidal inertia for each assembly.
% The three arm masses match mass, first axial moment and polar inertia about q3.
if nargin < 1, m_payload = 1.0; end
here = fileparts(mfilename('fullpath'));
src = fullfile(here,'Fusion','v3','step7_bodies','mass_properties.json');
data = jsondecode(fileread(src));
B = data.subassemblies;
column = B.D_02; slider = B.D_03; link = B.D_04; pen = B.D_05;
geom = robot_params();
length3 = geom.L3/1000;
length4 = geom.L4/1000;
D.source = string(src);
D.g = 9.81;
D.J1 = column.I_com(2,2) + column.mass*sum(column.com_m([1 3]).^2) ...
     + slider.I_com(2,2) + slider.mass*slider.com_m(3)^2;
D.m_slider = slider.mass;
D.s_slider = slider.com_m(1);
D.m_q3body = link.mass;
D.m_tool = pen.mass;
D.m_payload = m_payload;
x = [0 length3/2 length3];
Izz = link.I_com(3,3) + link.mass*sum(link.com_m(1:2).^2);
arm = [ones(1,3); x; x.^2] \ [link.mass; link.mass*link.com_m(1); Izz];
m_mid = pen.mass*pen.com_m(1)/(length4/2);
tool = [pen.mass-m_mid; m_mid];
assert(all([arm; tool] >= 0), 'CAD inertia cannot be represented by these point masses');

% Point masses. Each row of pts says where the point is: a3 = fraction of the q3 link,
% a4 = fraction of the tool, s = offset along the rail from the q3 joint [m] (slider only).
D.names   = ["q3 joint", "middle of the q3 link", "q4 joint", "middle of the tool", "pen tip (TCP)", "slider (its CoM)"];
D.pts     = [0 0 0;  0.5 0 0;  1 0 0;  1 0.5 0;  1 1 0;  0 0 D.s_slider];
D.i_tcp    = 5;          % row of the pen tip: the payload is there
D.i_slider = 6;          % row of the slider: it does not turn with q3
D.m = [arm(1);
       arm(2);
       arm(3) + tool(1);
       tool(2);
       D.m_payload;
       D.m_slider];
end
