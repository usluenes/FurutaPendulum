clc; clear;
%% Pendulum Parameters
% 0 - Arm
% 1 - Pendulum 
I0=1.75*10^-2;          %Inertia of the arm (kg*m^2)
L0=0.215;               %Total length of the arm  (m)
m1=5.38*10^-2;          %Mass of the pendulum (kg)
L1=0.113;               %Distance to the center of gravity of the pendulum (m)
J1=1.98*10^-4;          %Inertia of the pendulum around its center of gravity (kg*m^2)
%tau=0;                 %Input torque applied on the arm
g = 9.81;               %Gravity (m/s^2)
%% Simulation parameters
Theta0_init = -pi/2;    %Rotational angle of the arm
dTheta0_init = 0;       %Angular velocity of the arm
Theta1_init = 2.5*pi/3; %Rotational angle of the pendulum
dTheta1_init = 0;       %Angular velocity of the pendulum

Ts = 0.001;
dtDisc = 0.01;
Reference = [0 0 0 0];

StepX = 10;
distrub = 12;
disturb = distrub*pi/180;

%% D(q)ddtheta + C(q)dtheta + g(q) = F 
Dq = [I0+m1*(L0^2+L1^2*sin(Theta1_init)^2) m1*L1*L0*cos(Theta1_init); m1*L1*L0*cos(Theta1_init) J1+m1*L1^2];
DqPosDef = Dq(1,1)>0 &&  det(Dq)>0 %1  D(q) is positive definite 

%% Linearization
X1=-((m1^2)*(L1^2)*L0*g)/(I0*(J1+m1*(L1^2))+J1*m1*(L0^2));
Y1=((I0+m1*(L0^2))*m1*L1*g)/(I0*(J1+m1*(L1^2))+J1*m1*(L0^2));
X2=(J1+m1*(L1^2))/(I0*(J1+m1*(L1^2))+J1*m1*(L0^2));
Y2=(-m1*L1*L0)/(I0*(J1+m1*(L1^2))+J1*m1*(L0^2));
A=[0 1 0 0; 0 0 X1 0; 0 0 0 1; 0 0 Y1 0];
B=[0; X2; 0; Y2];
%C = eye(4);
D = 0;
%C=[1 0 0 0; 0 0 1 0];
C=[1 0 1 0];
% D=[0];

controllability=det(ctrb(A, B))>0 %Linearized model is controllable
observability=det(obsv(A, C))>0 %Linearized model is controllable

% State Feedback Control with 'lqr' Command
Q1 = [0.1 0 0 0;
     0 0.01 0 0;
     0 0 100 0;
     0 0 0 10];
R1 = 10;
[K1, ~, E] = lqr(A,B,Q1,R1);

% State Feedback Control with Pole Placement
K2 = place(A,B,[-5 -4 -2+2j -2-2j]);

% State Feedback Control with 'lqr' Command
R2 = 1;
Q2=[1 0 0 0;
    0 10 0 0;
    0 0 1000 0;
    0 0 0 10];
[K3, ~, E] = lqr(A,B,Q2,R2);

% State Feedback Gains (different source)
K4 = [-0.1000   -1.1155  -33.1974   -2.8474];

