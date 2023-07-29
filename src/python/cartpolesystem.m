% change this to increase/decrease number of poles/inverted pendulums
num_poles = 2;

% Removed symbolic variables for the constants
pole_as = sym("a",[1 num_poles]);
pole_ls = sym("l",[1 num_poles]);
pole_ms = sym("m",[1 num_poles]);
pole_ds = sym("d",[1 num_poles]);
pole_Js = sym("J",[1 num_poles]);

% Added constants with values of 1
% pole_as = 1*ones(1,num_poles);
% pole_ls = 1*ones(1,num_poles);
% pole_ms = 1*ones(1,num_poles);
% pole_ds = 1*ones(1,num_poles);
% pole_Js = 1*ones(1,num_poles);

% Create symbolic variables
syms m_c g
syms t s(t) tau
syms theta(t) [1 num_poles] % create time dependent thetas

% give time dependent variables lables
s = s(t); 
thetas = theta(t);
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
T_c = 1/2*m_c*diff(s,t)^2;
T_ps = 1/2*pole_ms.*(d_pole_pc1s.^2+d_pole_pc2s.^2)+1/2*pole_Js.*diff(thetas,t).^2;

% create potential energy
V = sum(g*pole_ms.*pole_pc2s);

% create dissipation function
d_thetas_extended = [0,diff(thetas,t)];
R = (1/2*pole_ds.*(d_thetas_extended(2:end)-d_thetas_extended(1:end-1)).^2);

% Add equation for tau
L = T_c-V;
d_s = diff(s,t);
L_ds = diff(L,d_s);
lh = diff(L_ds,t) - diff(L,s) + diff(R,d_s);
rh = tau;
eqs = cat(1, eqs, lh==rh);

% Add equations for dd_thetas
for i = 1:num_poles
    L = T_ps(i)-V;
    d_theta_i = diff(thetas(i),t);
    theta_i = thetas(i);
    L_d_theta_i = diff(L,d_theta_i);
    lh = diff(L_d_theta_i,t) - diff(L,theta_i) + diff(R,d_theta_i);
    rh = 0;
    eqs = cat(1,eqs,lh==rh);
end

% create dd_thetas symbols and equations
dd_thetas = sym("dd_theta",[1 num_poles]);
eqs = cat(1,eqs,dd_thetas == diff(thetas,t,t));

% Show equations
% disp("Eqs:")
% for i = 1:num_poles+1
%     disp(eqs(i))
% end

% solutions
disp("Sols:")
iso = isolate(eqs(1),tau);
sols = [iso];
disp(rhs(iso))

for i = 1:num_poles
    iso = isolate(eqs(i+1),diff(thetas(i),t,t));
    for j = i-1:-1:1
        iso = subs(iso,diff(thetas(j),t,t),rhs(sols(j+1)));
    end
    sols = cat(1,sols,iso);
    disp(simplify(rhs(iso)))
end