function [a1, a2, a3] = inversepos (P, alfa)
%description:
%   function that given the position and orientation of the end effector
%   finds the angles of the 3 joints of a particular 3R system


%dades del sistem 3R particular
l1 = 0.62;
l2 = 0.57;
g1 = 0.1; 
g2 = 0.2;
g3 = 0.3;

%per utilitzar les expressions que hi ha en les transparències del tema 1
%sumem g2 en l'eix y i restem g3 en l'eix x de la posició de l'end effector i
%utilitzem g1 com si fos a34
x = P(1) - g3; %posició end effector
y = P(2) + g2;


%alfa2
d_inv = 2*l1*l2;
f_inv = (x - g1*cos(alfa))^2 + (y - g1*sin(alfa))^2 - l1^2 - l2^2;

a2 = acos(f_inv/d_inv);
%Aixo tambe té de solució l'angle en negatiu que és el que volem perquè
%estem en situació de elbow down


%alfa1
A = l1 + l2 * cos(a2);
B = l2 * sin(a2);

E = x - g1 * cos(alfa);
F = y - g1 * sin(alfa);

% -B * s1 + A *c1 = E  // A * s1 + B *s1 = F
M1 = [-B, A; A, B]; 
M2 = [E; F]; 

%[X] = linsolve(M1, M2);
X=M1\M2;

sin1=X(1);
cos1=X(2);
a1 = - acos(cos1);

%alfa3
a3 = alfa - a1 - a2;

end
