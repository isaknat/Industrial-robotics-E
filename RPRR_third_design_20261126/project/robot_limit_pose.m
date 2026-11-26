function [q, interval] = robot_limit_pose(request,L,drum)
% Clamp q2 when jogging a requested shoulder/wrist pose. Angles in radians.
% Writing uses robot_coupled_limits directly so a blocked TCP path is reported.
assert(size(request,1)==4 && all(isfinite(request),'all'));
if any(request([1 3 4],:)<L.qmin([1 3 4]) | request([1 3 4],:)>L.qmax([1 3 4]),'all')
    error('Rotary joint outside its travel');
end
[hi,floor_mm,lo] = robot_coupled_limits(request(3,:),request(4,:),L,drum);
if any(hi<lo | floor_mm<=0)
    error('No slider position clears the drum for this orientation');
end
q=request;
q(2,:) = min(max(q(2,:),lo),hi);
interval=[lo;hi];
end
