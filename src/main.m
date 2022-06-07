% FILE:         assignment_Q1.m
% DESCRIPTION:  Developing Controller for WM363 Assignment
% AUTHOR:       James Bennion-Pedley
% DATE CREATED: 28/05/2022

%------------------------------------------------------------------------------%

close all; clc; clear;

%----------------------------------- Config -----------------------------------%

D6 = 5;
D7 = 9;

% Constants
c.M = (5 + 0.1*D6)*10^4;          % kg
c.J = (225 + 0.01*D7)*10^3;       % Nms^2
c.b_x = 5000;                     % Nsm^-1
c.b_y = 4790;                     % Nsm^-1
c.l = 15;                         % m
c.T_0 = 354650*(1 + 0.01*D6*D7);  % N
c.K_x = 0.1;                      % radsm^-1
c.K_y = -50;                      % radsm^-1

%----------------------------------- Model ------------------------------------%

% Initialise class with 1 input (m), 4 state (n), 1 output (p)
m = MimoControl([1, 4, 1], c);

m.ISOForm = [
   -sin(m.Q(3) - m.C.K_x*m.Q(1) - m.U(1))*((m.C.T_0 + m.C.K_y*m.Q(2))/m.C.M) - m.C.b_x*m.Q(1)/m.C.M;
    cos(m.Q(3) - m.C.K_x*m.Q(1) - m.U(1))*((m.C.T_0 + m.C.K_y*m.Q(2))/m.C.M) - m.C.b_y*m.Q(2)/m.C.M;
    m.Q(4);
    ((m.C.T_0 + m.C.K_y*m.Q(2))/m.C.J)*m.C.l*sin(m.C.K_x*m.Q(1) + m.U(1));
    m.Q(3);
];

% Set our target equilibrium points
q_eq = [
    -72.192;
    75.357;
    pi/4;
    0;
];

u_eq = [
    7.2192;
];

m.setEquilibriumPoints(q_eq, u_eq);

%--------------------------------- Controller ---------------------------------%

% METHOD 1 - Classical Controller
p = m.plantTransferFcn();

syms s;
c1 = 15*(1+0.5*s)/(1+0.2*s);
c2 = 1/(s*(1+0.29*s)*(1+0.17*s));

c_inner = m.applyController(p, c1);
c_outer = m.applyController(c_inner, c2);

f = Figure();
step(m.sym2tf(c_outer));
f.Title = "Step Response for Closed-Loop Classical Controller";
grid on;

% METHOD 2 - Full-State Feedback
e = [-0.1, -0.08, (-0.6+0.5i), (-0.6-0.5i)];
K = m.stateFeedbackMatrix(e);
G = m.stateObserverMatrix(e);
Kr = m.correctDCGain(K);

m.plotStepResponse(K, Kr);
grid on;

%---------------------------------- Simulink ----------------------------------%

Sim_A = m.EquilibriumStateSpace.A;
Sim_B = m.EquilibriumStateSpace.B;
Sim_C = m.EquilibriumStateSpace.C;

% ClassicalController.slx
[n, d] = numden(c1);
Sim_c1_n = sym2poly(n);
Sim_c1_d = sym2poly(d);
[n, d] = numden(c2);
Sim_c2_n = sym2poly(n);
Sim_c2_d = sym2poly(d);

% StateSpaceController.slx
Sim_K = K;
Sim_Kr = Kr;
Sim_G = G;
