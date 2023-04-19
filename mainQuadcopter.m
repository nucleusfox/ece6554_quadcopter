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
               'Dt',    zeros(3), ...
               'Dw',    zeros(3), ...
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
robo_est = quadcopter(p_est);

% Use the estimated parameters to build linear controller
q = 3.5;
%C = [eye(3), zeros([3 9])];
%Q = q*(C'*C)
Q = blkdiag(q*eye(6), 1.110822*eye(6));
%Q = blkdiag(eye(6), eye(6));
R = eye(4);
K = robo_est.setLinearGain(Q,R);

roboDMRAC = qcDMRAC(robo,robo_est);

xref = @(t) zeros([12 1]);
u = @(t,x) roboDMRAC.DMRAC(t,x,xref(t));
%u = @(t,x) robo.linearController(t,x,xref);
%u = @(t, x) zeros([4 1]);

x0 = [-0.5; 0.2; 1; 0;0;0; 0;0;0; 0;0;0];
Kx0 = zeros([12 4]);
Kr0 = zeros([12 4]);

tspan = [0, 25];
[tSim, xSim, xdotSim, uSim] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, u, xref);
%[tSim, xSim, xdotSim, uSim] = robo.runSim(tspan, x0, u);

%figure();
%subplot(211);
%plot(tSim,xSim(:,1:3));
%legend('x','y','z');
%subplot(212);
%plot(tSim,xSim(:,4:6));
%legend("$R_x$","$R_y$","$R_z$");


clf;
figure(1);
  subplot(2,1,1);
  plot(tSim, xSim(:,1:3), '.');
  xlabel('t');
  ylabel('q');
  %title('Coords', Interpreter='latex');
  legend('x', 'y', 'z');
  hold on;
  grid on;

  subplot(2,1,2);
  plot(tSim, xSim(:,13:16), '.');
  xlabel('t');
  ylabel('$q_m$', "Interpreter", "latex");
  %title('Coords', Interpreter='latex');
  legend('$x_m$', '$y_m$', '$z_m$', "Interpreter", "latex");
  hold on;
  grid on;

figure(2);
%figure(1);
%%subplot(411);
%%plot(tSim,xSim(:,1:3));
%%legend('x','y','z');
%plot(tSim,xSim(:,3), '-', 'LineWidth', 2);
%hold on;
%grid on;
%legend('z');


subplot(311);
plot(tSim,xSim(:,4:6), '.');
hold on;
grid on;
legend("$R_x$","$R_y$","$R_z$");

subplot(312);
plot(tSim,xSim(:,7:9), '.');
%legend("$\dot{R_x}$","$\dot{R_y}$","$\dot{R_z}$", "Interpreter", "latex");
legend("$v_x$","$v_y$","$v_z$", "Interpreter", "latex");
hold on;
grid on;

subplot(313);
plot(tSim,xSim(:,10:12), '.');
legend("$\omega_x$","$\omega_y$","$\omega_z$");
hold on;
grid on;
