%Exercici cinemàtica: Moviment del braç mecànic 3R


%tenim la pose inicial, la final i el twist
P_start = [0.9; -0.2]; %m
P_end = [0.9; -0.7]; %m
alfa = 0; %rad

%i la velocitat a la que volem que es mogui el braç (volem que la orientació sigui constant) 
v=-0.5; %m/s
T = [0; v; 0]; %el twist del robot és la velocitat v en l'eix y


%% 0. parametres del sistema
deltat= 0.015; %s

l1 = 0.62;  %m
l2 = 0.57 ; %m

g1 = 0.1; %m
g2 = 0.2; %m
g3 = 0.3; %m

% #iteracions per arribar fins a P_end
n = round( (P_end(2)-P_start(2))/(v*deltat) );


%% 1. inicialitzem tots els elements que necessitarem

    %r_i: posició del joint
r1 = []; %matriu de 2 files(x,y) on cada columna és una nova iteració
r2 = [];
r3 = [];
    %alpha_i: angle del joint
a1 = []; %vector (cada columna és una nova iteració)
a2 = [];
a3 = [];
    %gamma: velocitat de rotació de cada joint
gamma = []; %matriu de 3 files (una per joint) on cada columnaés una nova iteració
    %P: pose end effector, com que la orientació sempre serà la mateixa (0)
    %només ens quedarem amb la posició
P = []; %matriu de 2 files (x,y) on cada columna és una nova iteració
        %P(:,1) ha de ser P_start i P(:,end) ha de ser P_end


%% 2. trobem posicio joints (angles) amb la inverse pos
[a1(1), a2(1), a3(1)] = inversepos (P_start, alfa);

    %r_i és la posició del CIR de cada joint 
r1(:,1) = [0; 0];             
r2(:,1) = r1( :, 1) + l1 * [ cos( a1(1) ); sin( a1(1) )];
r3(:,1) = r2( :, 1) + l2 * [ cos( a1(1)+a2(1) ); sin( a1(1)+a2(1) )];

P(:,1) = r3( :, 1) + [g1+g3;-g2];
%if P_test is not equal to P_start anem malament


%%% Part 1:     Plot of the 3R motion

%A la figura 1 hi dibuixarem el gràfic del moviment del braç robòtic
%Comencem ara dibuixant la configuració inicial
figure(1)
hold on
title('3R Motion')
xlabel('x(m)')
ylabel('y(m)')

%   bodies (black)
plot([r1(1,1),r2(1,1)],[r1(2,1),r2(2,1)],'-k','LineWidth',5)
plot([r2(1,1),r3(1,1)],[r2(2,1),r3(2,1)],'-k','LineWidth',5)
plot([r3(1,1),r3(1,1)+g1],[r3(2,1),r3(2,1)],'-k','LineWidth',3)
plot([r3(1,1)+g1,r3(1,1)+g1],[r3(2,1),r3(2,1)-g2],'-k','LineWidth',3)
plot([r3(1,1)+g1,r3(1,1)+g1+g3],[r3(2,1)-g2,r3(2,1)-g2],'-k','LineWidth',3)

%   joints (red)
plot(r1(1,1),r1(2,1),'or','MarkerFaceColor','r')
plot(r2(1,1),r2(2,1),'or','MarkerFaceColor','r')
plot(r3(1,1),r3(2,1),'or','MarkerFaceColor','r')

axis([-0.2 1 -1 0.2])
grid on
hold off

%%%%%%%% Comencem les iteracions 

for k=1:n

    
%% 3. trobem el jacobià

    J = jacobian( r1(:,end), r2(:,end), r3(:,end));


%% 4. amb el jacobià i el twist (que en aquest cas és constant) trobem les
%velocitats dels joints (gamma) ambal IIKP
    gamma(:,end+1) = inversekin(J, T);


%% 5. Amb les velocitats dels joints podem trobar la nova posicio dels joints
%(theta_nova=theta_vella + deltat*gamma)
    a1(end+1) = a1(end) + deltat*gamma(1,end);
    a2(end+1) = a2(end) + deltat*gamma(2,end);
    a3(end+1) = a3(end) + deltat*gamma(3,end);


%% 6. trobem les noves posicions per poder fer els gràfics
    r1(:,end+1) = r1(:,end);             %r_i és la posició del CIR de cada joint 
    r2(:,end+1) = r1(:,end) + l1*[cos(a1(end)); sin(a1(end))];
    r3(:,end+1) = r2(:,end) + l2*[cos(a1(end)+a2(end)); sin(a1(end)+a2(end))];
    
    P(:,end+1) = r3(:, end) + [g1+g3;-g2];

%plot
    figure(1)
    axis([-0.2 1 -1 0.2])

%   bodies (black)
    plot([r1(1,end),r2(1,end)],[r1(2,end),r2(2,end)],'-k','LineWidth',5)
    hold on
    title('3R Motion')
    xlabel('x(m)')
    ylabel('y(m)')
    plot([r2(1,end),r3(1,end)],[r2(2,end),r3(2,end)],'-k','LineWidth',5)
    plot([r3(1,end),r3(1,end)+g1],[r3(2,end),r3(2,end)],'-k','LineWidth',3)
    plot([r3(1,end)+g1,r3(1,end)+g1],[r3(2,end),r3(2,end)-g2],'-k','LineWidth',3)
    plot([r3(1,end)+g1,r3(1,end)+g1+g3],[r3(2,end)-g2,r3(2,end)-g2],'-k','LineWidth',3)

%   joints (red)
    plot(r1(1,end),r1(2,end),'or','MarkerFaceColor','r')
    plot(r2(1,end),r2(2,end),'or','MarkerFaceColor','r')
    plot(r3(1,end),r3(2,end),'or','MarkerFaceColor','r')
    
    axis([-0.2 1 -1 0.2])
    grid on
    hold off

end


%%% Part 2:   Plot of joint speeds versus time
t = linspace(deltat,n*deltat,n);
figure(2)
hold on
grid on
title(['Joint speeds'])
xlabel(['t(s)'])
ylabel(['speed(rad/s)'])
axis([0 n*deltat -1.5 1.5])
plot(t, gamma(1,:),'r','LineWidth',2)
plot(t, gamma(2,:),'b','LineWidth',2)
plot(t, gamma(3,:),'g','LineWidth',2)
hold off
legend('w1','w2','w3')
