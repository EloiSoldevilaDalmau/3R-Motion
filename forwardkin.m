function T = forwardkin (J, gamma)
%description:
%   function that given the velocity of the end effector and the jacobian matrix finds the velocity of the joints for any system
%inputs:
%   J (jacobian matrix): 3x3 matrix [S1 S2 S3] where
%       Si=[yi;-xi;1]=[vx/w;vy/x;1] (for revolution joints
%   gamma (velocity of the joints): column vector of three elements [w1;w2;w3]
%outputs
%   T (twist): column vector of three elements   

T = J * gamma;

end