% change this to increase/decrease number of poles/inverted pendulums
num_poles = 2;

% Removed symbolic variables for the constants
pole_ms = sym("m",[1 num_poles]);
pole_ls = sym("l",[1 num_poles]);
pole_as = sym("a",[1 num_poles]);
pole_ds = sym("d",[1 num_poles]);
pole_Js = sym("J",[1 num_poles]);

% Create symbolic variables
syms m_c g
syms t calc_s(t) tau
syms calc_theta(t) [1 num_poles] % create time dependent thetas

% give time dependent variables lables
s = calc_s(t); 
thetas = calc_theta(t);
eqs = [];

% create vectors from the origin to the center of gravity of each pole
pole_pc1s = s.*ones(1,num_poles)-pole_as.*sin(thetas);
pole_pc2s = pole_as.*cos(thetas);

for i = 1:num_poles
    prev_1 = 0;
    prev_2 = 0;
    for j = 1:i-1
        prev_l = pole_ls(j);
        prev_theta = thetas(j);
        prev_1 = prev_1 - prev_l*sin(prev_theta);
        prev_2 = prev_2 + prev_l*cos(prev_theta);
    end
    pole_pc1s(i) = pole_pc1s(i)+prev_1;
    pole_pc2s(i) = pole_pc2s(i)+prev_2;
end

% time derivatives of pole origin to centre of gravity vectors
d_pole_pc1s = diff(pole_pc1s,t);
d_pole_pc2s = diff(pole_pc2s,t);

% create kinetic energies
T = 0.5*m_c*diff(s,t)^2;
T = T + sum(0.5*pole_ms.*(d_pole_pc1s.^2+d_pole_pc2s.^2)+0.5*pole_Js.*diff(thetas,t).^2);

% create potential energy
V = sum(g*pole_ms.*pole_pc2s);

% create dissipation function
d_thetas_extended = [0,diff(thetas,t)];
R = (0.5*pole_ds.*(d_thetas_extended(2:end)-d_thetas_extended(1:end-1)).^2);

% Add equation for tau
L = T-V;
d_s = diff(s,t);
lh = diff(diff(L,d_s),t) - diff(L,s) + diff(R,d_s);
rh = tau;
eqs = cat(1, eqs, lh==rh);

% Add equations for dd_thetas
for i = 1:num_poles
    theta_i = thetas(i);
    d_theta_i = diff(theta_i,t);
    lh = diff(diff(L,d_theta_i),t) - diff(L,theta_i) + diff(R,d_theta_i);
    rh = 0;
    eqs = cat(1,eqs,lh==rh);
end

for i = 1:num_poles+1
    disp(eqs(i))
end

% Subsitute time dependent variables
syms s d_s dd_s
syms theta [1 num_poles]
syms d_theta [1 num_poles]
syms dd_theta [1 num_poles]

old_vars = [diff(calc_s(t),t,t), diff(calc_s(t),t), calc_s(t)];
old_vars = [old_vars, diff(calc_theta(t),t,t), diff(calc_theta(t),t), calc_theta(t)];
new_vars = [dd_s, d_s, s];
new_vars = [new_vars, dd_theta, d_theta, theta];

for i = 1:num_poles+1
    eqs(i) = subs(eqs(i), old_vars, new_vars);
end

% Solve for dd_theta and stuff
sol_vars = cat(num_poles, tau, dd_theta);
sols = solve(eqs, sol_vars,"IgnoreAnalyticConstraints",true);

disp("sols:")
disp(sols)