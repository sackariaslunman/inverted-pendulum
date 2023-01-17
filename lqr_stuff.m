
% Example A and B taken from cart-pole problem
% Cart: m = 0.5, u_c = 0.05
% Motor: Ra = 0.05, Jm = 0.05, Bm = 0.01, K = 0.5, r = 0.05
% Pole 1: m = 0.2, l = 0.2, u_p1 = 0.005

load("AB.mat");

Va_max = 24;

A = double(A);
B = double(B);
disp(A)

C = eye(6);

D = zeros(6,1);

disp("Is system stable? Eigs: ")
disp(eigs(A))
disp("Is system controllable? Rank:")
disp(rank(ctrb(A,B)))
disp("Is system observable? Rank:")
disp(rank(obsv(A,C)))

Q = diag([1 1 1 1 1 1]);

R = [0.1;];

[K_lqr, S, E] = lqr(A, B, Q, R);

disp("K_lqr")
disp(K_lqr)

K_r = 1/(D+C*inv(-A+B*K_lqr)*B);
disp("K_r")
disp(size(K_r))

% no controller
% K_lqr = zeros(size(K_lqr));
% K_r = zeros(size(K_r));

x0 = [0; 0; -15/180*pi; 0; -15/180*pi; 0;]; 

t_final = 30;

sim("sim_model.slx")

t = ans.sim_X.time;
x1 = ans.sim_X.signals.values(:,1);
x2 = ans.sim_X.signals.values(:,2);
x3 = ans.sim_X.signals.values(:,3);
x4 = ans.sim_X.signals.values(:,4);
u1 = ans.sim_U.signals.values(:,1);

x5 = ans.sim_X.signals.values(:,5);
x6 = ans.sim_X.signals.values(:,6);

% plot
close all

figure
subplot(7,1,1)
plot(t, x1, "LineWidth", 3)
grid on
legend("x")

subplot(7,1,2)
plot(t, x2, "LineWidth", 3)
grid on
legend("d_x")

subplot(7,1,3)
plot(t, x3, "LineWidth", 3)
grid on
legend("theta1")

subplot(7,1,4)
plot(t, x4, "LineWidth", 3)
grid on
legend("d_theta1")

subplot(7,1,5)
plot(t, u1, "LineWidth", 3)
grid on
legend("Va")

subplot(7,1,6)
plot(t, x5, "LineWidth", 3)
grid on
legend("theta2")

subplot(7,1,7)
plot(t, x6, "LineWidth", 3)
grid on
legend("d_theta2")


