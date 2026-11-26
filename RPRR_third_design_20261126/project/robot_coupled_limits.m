function [q2max, floor_clearance, q2min] = robot_coupled_limits(q3,q4,L,drum)
% Upper slider position [mm] inside the drum for a shoulder/wrist posture.
% SI angles, millimetre lengths. The envelope covers the fork, wrist pulley,
% long belt, pen carrier and retracted pen. The tip may use its 3 mm spring travel.
% The drum is rotationally symmetric, so this bound is independent of q1.
q3 = q3(:).'; q4 = q4(:).';
q2max = repmat(L.qmax(2),size(q3));
q2min = repmat(L.qmin(2),size(q3));
floor_clearance = inf(size(q3));
    function include(x,y,z,pen)
        if pen
            pitch = q3+q4;
            dx = L.L3*cos(q3)+x*cos(pitch)-y*sin(pitch);
            h = L.h+L.L3*sin(q3)+x*sin(pitch)+y*cos(pitch);
        else
            dx = x*cos(q3)-y*sin(q3);
            h = L.h+x*sin(q3)+y*cos(q3);
        end
        inside = h>=0 & h<=drum.H;
        wall_radius = drum.R0+drum.A*sin(2*pi*drum.k*h/drum.H);
        allowed = sqrt(max(0,(wall_radius-0.35).^2-z^2))-dx;
        q2max(inside) = min(q2max(inside),allowed(inside));
        floor_clearance = min(floor_clearance,h);
    end
% Axial stations and both transverse extrema bound the square tube. The
% 0.35 mm radial allowance covers sinusoidal interpolation over 10 mm spans.
for x = unique([-20:10:320,304])
    lower = -20+max(0,x-304)*13.5/16;
    for y = [lower,lower/2,0,10,20], include(x,y,20,false); end
end
for x = 0:10:300
    r = (20+x/15)*3/(2*pi)+1.1;
    for y = [-r,0,r], include(x,y,45,false); end
end
% Circular wrist flange and hollow slide carrier.
for a = linspace(0,2*pi,65)
    include(300+20.5*cos(a),20.5*sin(a),46,false);
end
for x = -8:2:26
    for y = [-8,0,8], include(x,y,8,true); end
end
for x = 23:2:47
    r = min(6,(47-x)*6/8);
    for y = [-r,0,r], include(x,y,r,true); end
end
% Where the lower tube edge crosses the deck or base top, it must lie
% beyond the rear of the clearance slot or outside the round enclosure.
for obstacle = [181,38; 83,165].'
    h = obstacle(1)-L.h; edge = obstacle(2);
    falling = sin(q3)<0 & 320*sin(q3)-20*cos(q3)<h;
    crossing = (h*cos(q3(falling))+20)./sin(q3(falling));
    q2min(falling) = max(q2min(falling),edge+0.5-crossing);
end
end
