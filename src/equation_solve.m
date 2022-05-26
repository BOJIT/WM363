% FILE:         equation_solve.m
% DESCRIPTION:  Solve intersection points for equilibrium condition
% AUTHOR:       James Bennion-Pedley
% DATE CREATED: 25/05/2022

%------------------------------------------------------------------------------%

close all; clc; clear;

%----------------------------------- Config -----------------------------------%

D6 = 5;
D7 = 9;

M = (5 + 0.1*D6)*10^4;          % kg
J = (225 + 0.01*D7)*10^3;       % Nms^2
b_x = 5000;                     % Nsm^-1
b_y = 4790;                     % Nsm^-1
l = 15;                         % m
T_0 = 354650*(1 + 0.01*D6*D7);  % N
K_x = 0.1;                      % radsm^-1
K_y = -50;                      % radsm^-1

%-------------------------------- Entry Point ---------------------------------%

bound = 150;
step = linspace(-bound, bound, 500);
func = @(x, y) ((K_y^2-b_y^2)*y.^2 + (2*T_0*K_y)*y + T_0^2 - b_x^2*x.^2);
func2 = @(x, y) ((-b_x/b_y)*x - y);

f = Figure();

h = zeros(length(step));
for i = 1:length(step)
    for j = 1:length(step)
        h(i, j) = func(step(i), step(j));
    end
end

% figure;
p = pcolor(f.Axes(1), step, step, h);
set(p, 'EdgeColor', 'none');
colorbar();

fimplicit(f.Axes(1), func, [-bound, bound, -bound, bound]);
fimplicit(f.Axes(1), func2, [-bound, bound, -bound, bound]);

f.Title = "Implicit solution for Equilibrium Points of MIMO System";
f.XLabel = "$q_1 (\dot{x})$";
f.YLabel = "$q_2 (\dot{y})$";

% Brute force solver
x0 = [0, 0, 0];
x = fsolve(@mimoSystem, x0);

disp("Brute Force Solution: q_1, q_2, u");
disp(x);

f.plot(x(1), x(2), '*r');

%------------------------------ Helper Functions ------------------------------%

% q_1 = q(1), q_2 = q(2), u = q(3)
function F = mimoSystem(q)
    D6 = 5;
    D7 = 9;

    M = (5 + 0.1*D6)*10^4;          % kg
    J = (225 + 0.01*D7)*10^3;       % Nms^2
    b_x = 5000;                     % Nsm^-1
    b_y = 4790;                     % Nsm^-1
    l = 15;                         % m
    T_0 = 354650*(1 + 0.01*D6*D7);  % N
    K_x = 0.1;                      % radsm^-1
    K_y = -50;                      % radsm^-1

    q_3 = pi/4;

    F(1) = -sin(q_3 - K_x*q(1) - q(3))*((T_0 + K_y*q(2))/M) - (b_x*q(1))/M;
    F(2) = cos(q_3 - K_x*q(1) - q(3))*((T_0 + K_y*q(2))/M) - (b_y*q(2))/M;
    F(3) = sin(K_x*q(1) + q(3))*((T_0 + K_y*q(2))/J)*l;
end
