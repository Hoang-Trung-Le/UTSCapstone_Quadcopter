function R = RotMat(ang)
%ROTMAT Summary of this function goes here
%   Detailed explanation goes here

R = rotz(ang(3))*roty(ang(2))*rotx(ang(1));

end

