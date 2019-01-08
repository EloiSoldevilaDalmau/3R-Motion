function gamma = inversekin(J, T)
%description:
%   function that given the jacobian and the twisan matrix finds the velocity of the joints of any system
%inputs:
%   J (jacobian matrix) 3x3 matrix [S1 S2 S3] where
%       Si=[yi;-xi;1]=[vx/w;vy/x;1] (for revolution joints
%   T (twist): column vector of 3 elements
%outputs:
%   gamma(velocities of joints) column vector of three elements [w1;w2;w3]
gamma = inv(J) * T;
end