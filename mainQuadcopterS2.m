%% Quadcopter 3d

if (~exist('prettyup'))
  prettyup = true;                  % Should environment be cleared out?
end                                 % Affects repeat runs. Set as you please.

if (prettyup)                       % If prettyup should be done, do so.
  clearEnv;
  prettyup = true;                  
end                                 % otherwise, environment stays as is.

% true parameters
param = struct('m',     0.6,...
               'I',     diag([2.5e-3, 2.5e-3, 3.8e-3]),...
               'L',     0.175,...
               'Dt',    1.5*eye(3), ...
               'Dw',    0.75*eye(3), ...
               'b',     0.9, ...
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
Q = blkdiag(q*eye(6), 1.110822*eye(6));
R = eye(4);
K = robo_est.setLQR(Q,R);
robo.setLinearGain(K);

roboDMRAC = qcDMRAC(robo,robo_est);
roboDMRAC.linDMRAC.GammaX = 100*eye(12);
roboDMRAC.linDMRAC.GammaR = 100*eye(12);
roboDMRAC.linDMRAC.GammaAlpha = 10;

xref = @(t) zeros([12 1]);
%xref = @(t) [0;0;sin(t/2);0;0;0;0;0;0.5*cos(t/2);0;0;0];
%xref = @(t) [cos(t/2);0;sin(t/2);0;0;0;-0.5*sin(t/2);0;0.5*cos(t/2);0;0;0];

u = @(t,x) roboDMRAC.DMRAC(t,x,xref(t));
%u = @(t,x) robo.linearController(t,x,xref);
%u = @(t, x) zeros([4 1]);

% initial conditions
x0 = [-2; 1; -4; 0;0;0; 0;0;0; 0;0;0];

Kx0 = -K';
Kr0 = K';

g = 9.81;
a0 = robo_est.param.m*g*[1;1;1;1]/(4*robo_est.param.b);

% run the robot sim
tspan = [0, 5];


[tSim1, xSim1, xdotSim1, uSim1] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, a0, u, xref);
qcDMRAC.plot(tSim1, xSim1)
qcDMRAC.plotU(tSim1, uSim1)
qcDMRAC.plotError(tSim1, xSim1)
qcDMRAC.plotK(tSim1, xSim1)

Kx0 = xSim1(end,25:72)';
Kr0 = xSim1(end,73:120)';
a0 = xSim1(end,121:124)';

[tSim2, xSim2, xdotSim2, uSim2] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, a0, u, xref);
qcDMRAC.plot(tSim2, xSim2)
qcDMRAC.plotU(tSim2, uSim2)
qcDMRAC.plotError(tSim2, xSim2)
qcDMRAC.plotK(tSim2, xSim2)

% running the robot with the linear controller designed with estimated
% parameters and no adaptation
roboDMRAC.linDMRAC.GammaX = zeros(12,12);
roboDMRAC.linDMRAC.GammaR = zeros(12,12);
roboDMRAC.linDMRAC.GammaAlpha = 0;

Kx0 = -K';
Kr0 = K';
g = 9.81;
a0 = robo_est.param.m*g*[1;1;1;1]/(4*robo_est.param.b);

u = @(t,x) roboDMRAC.DMRAC(t,x,xref(t));
[tSim3, xSim3, xdotSim3, uSim3] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, a0, u, xref);
qcDMRAC.plot(tSim3, xSim3)
qcDMRAC.plotU(tSim3, uSim3)
qcDMRAC.plotError(tSim3, xSim3)
qcDMRAC.plotK(tSim3, xSim3)

u = @(t,x) robo_est.linearController(t,x,xref);
[tSim, xSim, xdotSim, uSim] = robo.runSim(tspan, x0, u);