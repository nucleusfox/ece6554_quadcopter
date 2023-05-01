%% Quadcopter 3d

if (~exist('prettyup'))
  prettyup = true;                  % Should environment be cleared out?
end                                 % Affects repeat runs. Set as you please.

if (prettyup)                       % If prettyup should be done, do so.
  clearEnv;
  prettyup = true;                  
end                                 % otherwise, environment stays as is.

% true parameters
param = struct('m',     0.5,...
               'I',     diag([2.32e-3, 2.32e-3, 4e-3]),...
               'L',     0.175,...
               'Dt',    1.5*eye(3), ...
               'Dw',    0.75*eye(3), ...
               'b',     1, ...
               'Km',    1.5e-9,...
               'kF',    6.11e-8,...
               'gamma', 1.5e-9/6.11e-8,...
               'km',    20);
           
% estimated/ideal model parameters
p_est = struct('m',     0.5,...
               'I',     diag([2.32e-3, 2.32e-3, 4e-3]),...
               'L',     0.175,...
               'Dt',    zeros(3), ...
               'Dw',    zeros(3), ...
               'b',     1, ...
               'Km',    1.5e-9,...
               'kF',    6.11e-8,...
               'gamma', 1.5e-9/6.11e-8,...
               'km',    20);

robo = quadcopter(param);

roboNMPC = qcNMPC(robo);
%xref = @(t) [1; 1; 1; 0;0;0; 0;0;0; 0;0;0];
%xref = @(t) [2*cos(t/2); cos(t/3); 3*cos(t/2); 0;0;0; -sin(t/2);-sin(t/3)/3;-1.5*sin(t/2); 0;0;0];
%xref = @(t) [6*sin(t/3); -6*sin(t/3).*cos(t/3); 6*cos(t/3); 0;0;0; 0;0;0; 0;0;0];
xref = @(t) [0;0;sin(pi*t/5);0;0;0;0;0;0.2*pi*cos(pi*t/5);0;0;0];

%u = @(t,x) roboNMPC.NMPC(t,x,xref(t));

%x0 = [-0.5; 0.2; 1; 0;0;0; 0;0;0; 0;0;0];
x0 = zeros(12,1);

roboNMPC.setInitialX(x0);

tspan = [0, 10];
[tSim, xSim, xdotSim, uSim] = roboNMPC.runSim(tspan, x0, xref);

N = length(tSim);
rHistory = zeros(N,12);
for i = 1:N
    rHistory(i,:) = xref(tSim(i))';
end

clf;
figure(1);

  title('Coordinates and control', 'Interpreter','latex');
  subplot(2,2,1);
  plot(tSim, [xSim(:,1) rHistory(:, 1)], '.','LineWidth',2.0);
  xlabel('$t$', "Interpreter", "latex");
  ylabel('$q$', "Interpreter", "latex");
  legend('x', 'desired x');
  hold on;
  grid on;

  subplot(2,2,2);
  plot(tSim, [xSim(:,2) rHistory(:, 2)], '.','LineWidth',2.0);
  xlabel('$t$', "Interpreter", "latex");
  ylabel('$q$', "Interpreter", "latex");
  legend('y', 'desired y');
  hold on;
  grid on;

  subplot(2,2,3);
  plot(tSim, [xSim(:,3) rHistory(:, 3)], '.','LineWidth',2.0);
  xlabel('$t$', "Interpreter", "latex");
  ylabel('$q$', "Interpreter", "latex");
  legend('z', 'desired z');
  hold on;
  grid on;

  subplot(2,2,4);
  plot(tSim, uSim(:,1:4), '.','LineWidth',2.0);
  xlabel('$t$', "Interpreter", "latex");
  ylabel('$u$', "Interpreter", "latex");
  legend('$u_1$', '$u_2$', '$u_3$', '$u_4$', "Interpreter", "latex");
  hold on;
  grid on;

figure(2);
    subplot(311);
    plot(tSim,xSim(:,4:6), '.','LineWidth',2.0);
    xlabel('$t$', "Interpreter", "latex");
    ylabel('$R_x, R_y, R_z$', "Interpreter", "latex");
    hold on;
    grid on;
    legend("$R_x$","$R_y$","$R_z$");
    
    subplot(312);
    plot(tSim,xSim(:,7:9), '.','LineWidth',2.0);
    xlabel('$t$', "Interpreter", "latex");
    ylabel('$v_x, v_y, v_z$', "Interpreter", "latex");
    legend("$v_x$","$v_y$","$v_z$", "Interpreter", "latex");
    hold on;
    grid on;
    
    subplot(313);
    plot(tSim,xSim(:,10:12), '.','LineWidth',2.0);
    xlabel('$t$', "Interpreter", "latex");
    ylabel('$\omega_x,\omega_y, \omega_z$', "Interpreter", "latex");
    legend("$\omega_x$","$\omega_y$","$\omega_z$");
    hold on;
    grid on;


%ani = quadcopterAnimator(robo);

%ani.fig = figure(3); clf;
%ani.animate(tSim,xSim);
%ani.export_movie("demo", 12);

%figure(4);
%plot(tSim, transpose(uSim));
%legend({'$u_1$','$u_2$','$u_3$','$u_4$'}, 'Interpreter','latex');

%ani.fig = figure(5); clf;
%ani.animateInput(tSim,xSim,uSim);