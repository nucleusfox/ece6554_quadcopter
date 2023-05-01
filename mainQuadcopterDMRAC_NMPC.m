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
               'Dt',    4.5*eye(3), ...
               'Dw',    6.75*eye(3), ...
               'b',     1, ...
               'Km',    1.5e-9,...
               'kF',    6.11e-8,...
               'gamma', 1.5e-9/6.11e-8,...
               'km',    20);
           
% estimated/ideal model parameters
p_est = struct('m',     0.5,...
               'I',     diag([2.32e-3, 5.32e-3, 3e-3]),...
               'L',     0.175,...
               'Dt',    zeros(3), ...
               'Dw',    zeros(3), ...
               'b',     1.2, ...
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

roboDMRAC = qcDMRAC_NMPC(robo,robo_est);
roboDMRAC.linDMRAC.GammaX = 100*eye(12);
roboDMRAC.linDMRAC.GammaR = 100*eye(12);
roboDMRAC.linDMRAC.GammaAlpha = 10;

%xref = @(t) zeros([12 1]);
%xref = @(t) [6*cos(t/4); 3*sin(t/2); 3*cos(t/12); 0;0;0; -6/4*sin(t/4); 3/2*cos(t/2); -3/12*sin(t/12); 0;0;0];
%xref = @(t) [0;0;sin(t/2);0;0;0;0;0;0.5*cos(t/2);0;0;0];
xref = @(t) [cos(t/2);0;sin(t/2);0;0;0;-0.5*sin(t/2);0;0.5*cos(t/2);0;0;0];

u = @(t,x,uFF) roboDMRAC.DMRAC(t,x,xref(t),uFF);

% initial conditions
x0 = [-2; 1; -4; 0;0;0; 0;0;0; 0;0;0];

Kx0 = zeros(12,4);
Kr0 = zeros(12,4);
a0 = zeros(4,1);

% run the robot sim
tspan = [0, 50];


[tSim1, xSim1, xdotSim1, uSim1, umSim1] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, a0, u, xref);
qcDMRAC_NMPC.plot(tSim1, xSim1);
qcDMRAC_NMPC.plotU(tSim1, uSim1);
qcDMRAC_NMPC.plotError(tSim1,xSim1);
qcDMRAC_NMPC.plotK(tSim1, xSim1);

Kx0 = xSim1(end,25:72)';
Kr0 = xSim1(end,73:120)';
a0 = xSim1(end,121:124)';

[tSim2, xSim2, xdotSim2, uSim2, umSim2] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, a0, u, xref);
qcDMRAC_NMPC.plot(tSim2, xSim2);
qcDMRAC_NMPC.plotU(tSim2, uSim2);
qcDMRAC_NMPC.plotError(tSim2,xSim2);
qcDMRAC_NMPC.plotK(tSim2, xSim2);


% running the robot with the non-adaptive
% controller designed with estimated parameters
roboDMRAC = qcDMRAC_NMPC(robo,robo_est);
roboDMRAC.linDMRAC.GammaX = zeros(12,12);
roboDMRAC.linDMRAC.GammaR = zeros(12,12);
roboDMRAC.linDMRAC.GammaAlpha = 0;
Kx0 = zeros(12,4);
Kr0 = zeros(12,4);
a0 = zeros(4,1);

u = @(t,x,uFF) roboDMRAC.DMRAC(t,x,xref(t),uFF);
[tSim3, xSim3, xdotSim3, uSim3, umSim3] = roboDMRAC.runSim(tspan, x0, Kx0, Kr0, a0, u, xref);
qcDMRAC_NMPC.plot(tSim3, xSim3);
qcDMRAC_NMPC.plotU(tSim3, uSim3);
qcDMRAC_NMPC.plotError(tSim3,xSim3);
qcDMRAC_NMPC.plotK(tSim3, xSim3);