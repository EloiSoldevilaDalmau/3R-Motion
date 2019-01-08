function J = jacobian(j1,j2,j3)

%function that gives the jacobian matrix of a system of 3 revolution joints
%given the position (j_i) of each joint

%j_i (input): position of joint i (2 element vector)
%J(output): jacobian matrix (3x3)

v1 = [j1(2); -j1(1); 1]; v2 = [j2(2); -j2(1); 1]; v3 = [j3(2);-j3(1); 1];
J=[v1 v2 v3];

end