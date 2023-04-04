%% Quadcopter 3d

if (~exist('prettyup'))
  prettyup = true;                  % Should environment be cleared out?
end                                 % Affects repeat runs. Set as you please.

if (prettyup)                       % If prettyup should be done, do so.
  clearEnv;
  prettyup = true;                  
end                                 % otherwise, environment stays as is.


robo = quadcopter();


g = 9.81;
u = @(t,x) robo.param.m*g*[1.0101;1.0100;1.010;1.010]/4;

x0 = [-0.5;0.2;1; 0;0;0; 0;0;0; 0;0;0];

tspan = [0, 2];
[tSim, xSim, xdotSim, uSim] = robo.runSim(tspan, x0, u);

ani = quadcopterAnimator(robo);

ani.fig = figure(1); clf;
ani.animate(tSim,xSim);

figure(2);
plot(tSim, transpose(uSim));
legend({'$u_1$','$u_2$','$u_3$','$u_4$'}, 'Interpreter','latex');

ani.fig = figure(3); clf;
ani.animateInput(tSim,xSim,uSim);
