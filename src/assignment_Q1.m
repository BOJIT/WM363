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


% e = m.findEquilibriumPoints([
%     % Variables can only appear in a single condition!
%     m.C.T_0 + m.C.K_y*m.Q(2) ~= 0;
%     m.Q(3) == pi/4;
%     (m.C.K_x*m.Q(1) + m.U(1) < pi/2) | (m.C.K_x*m.Q(1) + m.U(1) > -pi/2);
% ]);


% Assume equilibrium points worked;
q_fake = [
    -72.192;
    75.357;
    pi/4;
    0;
];

u_fake = [
    7.2192;
];

m.setEquilibriumPoints(q_fake, u_fake);

%--------------------------------- Controller ---------------------------------%

% p = m.transferFcn(m.EquilibriumStateSpace);
%
% m.launchDesigner(p);

e = [-0.1, -0.08, (-0.6+0.5i), (-0.6-0.5i)];
% e = [-0.1, -0.08, (-5+0.5i), (-5-0.5i)];
K = m.stateFeedbackMatrix(e);
disp(K);

G = m.stateObserverMatrix(e);
disp(G);

Kr = m.correctDCGain(K);
m.plotStepResponse(K, Kr);

%------------------------------ Helper Functions ------------------------------%
