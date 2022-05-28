% FILE:         controller_design.m
% DESCRIPTION:  Controller design example script
% AUTHOR:       James Bennion-Pedley
% DATE CREATED: 05/05/2022

%------------------------------------------------------------------------------%

close all; clc; clear;

%----------------------------------- Config -----------------------------------%

% University ID
D6 = 5;
D7 = 9;

%------------------------------- Symbolic Maths -------------------------------%

syms M J b_x b_y l T_0 K_x K_y;
syms q_1 q_2 q_3 q_4 u;

q = [q_1, q_2, q_3, q_4];

f = [ ...
    -sin(q_3 - K_x*q_1 - u)*((T_0 + K_y*q_2)/M) - (b_x/M)*q_1, ...
    cos(q_3 - K_x*q_1 - u)*((T_0 + K_y*q_2)/M) - (b_y/M)*q_2, ...
    q_4, ...
    sin(K_x*q_1 + u)*l*((T_0 + K_y*q_2)/J) ...
];

A = jacobian(f, q);
B = jacobian(f, u);
C = [0, 0, 1, 0];
D = 0;

%---------------------------- Numeric Substitution ----------------------------%

% Constants
M = (5 + 0.1*D6)*10^4;          % kg
J = (225 + 0.01*D7)*10^3;       % Nms^2
b_x = 5000;                     % Nsm^-1
b_y = 4790;                     % Nsm^-1
l = 15;                         % m
T_0 = 354650*(1 + 0.01*D6*D7);  % N
K_x = 0.1;                      % radsm^-1
K_y = -50;                      % radsm^-1

% Equilibrium Point
q_1 = -72.192;                  % ms^-1
q_2 = 75.357;                   % ms^-1
q_3 = pi/4;                     % rad
q_4 = 0;                        % rads^-1
u = 7.2192;                     % rad

% Substitute to 4.d.p.
A_q = double(round(subs(A), 4));
B_q = double(round(subs(B), 4));

%------------------------------ Transfer Function -----------------------------%

[n, d] = ss2tf(A_q, B_q, C, D);

syms g;
g = poly2sym(n)/poly2sym(d);
gout = vpa(g, 3);
disp(latex(gout));

%----------------------------------- Output -----------------------------------%

disp("A Jacobian");
disp("---------------------------------------------------------");
for i = 1:numel(A)
    disp(latex(A(i)));
end

disp("---------------------------------------------------------");
disp(A_q);

disp("B Jacobian");
disp("---------------------------------------------------------");
for i = 1:numel(B)
    disp(latex(B(i)));
end

disp("---------------------------------------------------------");
disp(B_q);

%------------------------------ Helper Functions ------------------------------%
