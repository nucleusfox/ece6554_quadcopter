classdef qcDMRAC_NMPC < handle
    properties
        robot = quadcopter() % Actual quadcopter system
        model = quadcopter() % Ideal/model quadcopter system
        NMPC;
        
        tSim
        xSim % states are x, dx, th, dth, i_motor
        xdotSim
        uSim
        
        linDMRAC = struct('Am', zeros(12),...
                          'Bm', zeros([12 4]),...
                          'Pm', zeros(12),...
                          'Qm', eye(12),...
                          'GammaR', eye(12),...
                          'GammaX', eye(12),...
                          'GammaAlpha', 1,...
                          'GammaBeta', eye(12),...
                          'Bdir', zeros([12 4]))
                
    end
    
    methods
        
        function obj = qcDMRAC_NMPC(robot, model)
            if (nargin >= 1) 
                obj.robot = robot;
                if (nargin >= 2) 
                    obj.model = model;
                end
            end

            obj.NMPC = qcNMPC(model);

            [A,B] = obj.model.getLinearization();
            K = obj.model.linContr.K;
            obj.linDMRAC.Am = A-B*K;
            obj.linDMRAC.Bm = B*K;
            
            obj.linDMRAC.Pm = obj.model.linContr.P;
            obj.linDMRAC.Qm = obj.model.linContr.Q;
            
            [Bt,Ba] = obj.robot.Bfun(1);
            obj.linDMRAC.Bdir = [zeros([6 4]);Bt;Ba];
        end
        
        function [tSim, xSim, xdotSim, uSim, umSim] = runSim(self, tspan, x0, Kx0, Kr0, a0, u, xref)

            self.NMPC.xHistory = [x0;x0;Kx0(:);Kr0(:);a0]';
            tSim = [];
            xSim = [];
            umSim = [];

            self.NMPC.params.Duration = tspan(2);
            
            % Simulate the system.
            for k = 1:(self.NMPC.params.Duration/self.NMPC.params.Ts)
                % Compute model control move with reference previewing
                xk = self.NMPC.xHistory(k,13:24);
                uk = self.NMPC.uNMPC(k*self.NMPC.params.Ts,xk,xref);
                % Store control move
                self.NMPC.lastMV = uk;

                ODEFUN = @(t,x) self.dynamics(t, x, u(t,x,uk), uk, xref(t));
                [TOUT,XOUT] = ode45(ODEFUN,[0 self.NMPC.params.Ts], self.NMPC.xHistory(k,:)');

                self.NMPC.xHistory(k+1,:) = XOUT(end,:);

                tSim = cat(1,tSim,TOUT(2:end) + (k-1)*self.NMPC.params.Ts);
                xSim = cat(1,xSim,XOUT(2:end,:));
                umSim = cat(1,umSim,ones(length(TOUT)-1,1)*uk');
            end

            % Use simulation data to recover controls and diff eq.
            % Data is row-wise.
            num     = length(tSim);
            uSim    = zeros(num,4);
            xdotSim = zeros(size(xSim));
            
            for i = 1:num
                x = xSim(i,:)';
                t = tSim(i);
                um = umSim(i,:)';
                uSim(i,:) = u(t, x, um)';
                xdotSim(i,:) = self.dynamics(t, x, uSim(i,:)', um, xref(t))';
            end
            
            % Store variables in case needed for later.
            self.tSim = tSim;
            self.xSim = xSim;
            self.xdotSim = xdotSim;
            self.uSim = uSim;
        end
        
        function dx = dynamics(self, t, xvec, uvec, umvec, xref)
            dx = zeros([124 1]);
            xvec_x = xvec(1:12);
            xvec_m = xvec(13:24);
            
            % update system using actual robot dynamics
            dx(1:12) = self.robot.dynamics(t,xvec_x,uvec);
            % reference model with NMPC
            dx(13:24) = self.model.dynamics(t,xvec_m,umvec);
            
            % update adaptive gains
            f = (xvec_x - xvec_m)'*self.linDMRAC.Pm*self.linDMRAC.Bdir;
            dKx = -self.linDMRAC.GammaX*xvec_x*f;
            dKr = -self.linDMRAC.GammaR*xref*f;
            dAlpha = -self.linDMRAC.GammaAlpha*f;
            
            dx(25:72) = dKx(:);
            dx(73:120) = dKr(:);
            dx(121:124) = dAlpha';
        end
        
        function u = DMRAC(self, t, xvec, xref, uFF)
            g = 9.81;
            xvec_x = xvec(1:12);
            Kx = reshape(xvec(25:72),[12 4]);
            Kr = reshape(xvec(73:120),[12 4]);

            u = (Kx.')*xvec_x + (Kr.')*xref ...
                + xvec(121:124) + uFF;
        end        
    end
    
    methods (Static)
        function plot(tSim, xSim)
            f = figure();
            
            f.Position = [680, 558, 1120, 420];
            
              subplot(2,2,1);
              plot(tSim, xSim(:,1:3), '.');
              xlabel('t');
              ylabel('$q$',"Interpreter", "latex");
              %title('Coords', Interpreter='latex');
              legend('x', 'y', 'z');
              hold on;
              grid on;

              subplot(2,2,2);
              plot(tSim, xSim(:,13:15), '.');
              xlabel('t');
              ylabel('$q_m$', "Interpreter", "latex");
              legend('$x_m$', '$y_m$', '$z_m$', "Interpreter", "latex");
              hold on;
              grid on;
              
              subplot(2,2,3);
              plot(tSim,xSim(:,4:6), '.');
              xlabel('t');
              ylabel('$R$', "Interpreter", "latex");
              hold on;
              grid on;
              legend("$R_x$","$R_y$","$R_z$");
              
              subplot(2,2,4);
              plot(tSim, xSim(:,16:18), '.');
              xlabel('t');
              ylabel('$R_m$', "Interpreter", "latex");
              legend('$R_{x,m}$', '$R_{y,m}$', '$R_{z,m}$', "Interpreter", "latex");
              hold on;
              grid on;
        end

        function plotError(tSim, xSim)
            f = figure();
            
            f.Position = [680, 558, 560, 420];
            
              subplot(2,1,1);
              plot(tSim, xSim(:,1:3)-xSim(:,13:15), '.');
              xlabel('t');
              ylabel('$e_q$',"Interpreter", "latex");
              %title('Coords', Interpreter='latex');
              legend('x', 'y', 'z');
              hold on;
              grid on;
              
              subplot(2,1,2);
              plot(tSim,xSim(:,4:6)-xSim(:,16:18), '.');
              xlabel('t');
              ylabel('$e_R$', "Interpreter", "latex");
              hold on;
              grid on;
              legend("$R_x$","$R_y$","$R_z$");
        end

        function plotU(tSim,uSim)
            figure();
              plot(tSim, uSim, '.');
              xlabel('t');
              ylabel('$u$',"Interpreter", "latex");
              hold on;
              grid on;
        end

        function plotK(tSim,xSim)
            f = figure();

              f.Position = [680, 348, 560, 630];
            
              subplot(3,1,1);
              plot(tSim, xSim(:,25:72), '-');
              xlabel('t');
              ylabel('$K_x$',"Interpreter", "latex");
              %title('Coords', Interpreter='latex');
              hold on;
              grid on;
              
              subplot(3,1,2);
              plot(tSim,xSim(:,73:120), '-');
              xlabel('t');
              ylabel('$K_r$', "Interpreter", "latex");
              hold on;
              grid on;

              subplot(3,1,3);
              plot(tSim,xSim(:,121:124), '-');
              xlabel('t');
              ylabel('$\hat{\alpha}$', "Interpreter", "latex");
              hold on;
              grid on;
        end
    end
end
