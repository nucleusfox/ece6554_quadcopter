%% Quadcopter 3d

if (~exist('prettyup'))
  prettyup = true;                  % Should environment be cleared out?
end                                 % Affects repeat runs. Set as you please.

if (prettyup)                       % If prettyup should be done, do so.
  clearEnv;
  prettyup = true;                  
end                                 % otherwise, environment stays as is.


robo = quadcopter();

q = 10;
Q = blkdiag(q*eye(6),eye(6));
R = eye(4);
K = robo.getLinearGain(Q,R);
robo.setLinearGain(K);

g = 9.81;
%u = @(t,x) robo.param.m*g*[1.0101;1.0100;1.010;1.010]/4;

xref = @(t) zeros([12 1]);
u = @(t,x) robo.linearController(t,x,xref);

x0 = [-0.5;0.2;1; 0;0;0; 0;0;0; 0;0;0];

tspan = [0, 5];
[tSim, xSim, xdotSim, uSim] = robo.runSim(tspan, x0, u);

figure();
subplot(211);
plot(tSim,xSim(:,1:3));
legend('x','y','z');
subplot(212);
plot(tSim,xSim(:,4:6));
legend("$R_x$","$R_y$","$R_z$");

%ani = quadcopterAnimator(robo);

%ani.fig = figure(1); clf;
%ani.animate(tSim,xSim);

%figure(2);
%plot(tSim, transpose(uSim));
%legend({'$u_1$','$u_2$','$u_3$','$u_4$'}, 'Interpreter','latex');

%ani.fig = figure(3); clf;
%ani.animateInput(tSim,xSim,uSim);
