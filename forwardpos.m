function [P, alfa] = forwardpos (alpha1, alpha2, alpha3)
%description:
%   functin that given the angles of the different joints finds the pose of
%   the end effector for a particular 3R system
%inputs:
%   alpha_i: angle of joint i (scalar)
%outputs:
%   P: position of end effector (vector,  [x,y])
%   alfa: orientation end effector (scalar)

l1 = 0.62;  %m
l2 = 0.57 ; %m

g1 = 0.1;
g2 = 0.2;
g3 = 0.3;

x = l1 * cos (alpha1) + l2 * cos (alpha1 + alpha2) + g1 * cos (alpha1 + alpha2 + alpha3) + g3;
y = l1 * sin (alpha1) + l2 * sin (alpha1 + alpha2) + g1 * sin (alpha1 + alpha2 + alpha3) - g2;
P = [x;y]

% v0 = [0; 0];
% v1 = [0.62; 0];
% v2 = [0.57; 0];
% v3 = [0.1; 0];
% 
% R1 = [cos(alpha1) -sin(alpha1); sin(alpha1) cos(alpha1)];
% R2 = [cos(alpha1+alpha2) -sin(alpha1+alpha2); sin(alpha1+alpha2) cos(alpha1+alpha2)];
% R3 = [cos(alpha1+alpha2+alpha3) -sin(alpha1+alpha2+alpha3); sin(alpha1+alpha2+alpha3) cos(alpha1+alpha2+alpha3)];
% P = v0 + R1*v1 + R2*v2 + R3*v3;

alfa = alpha1 + alpha2 + alpha3;
end