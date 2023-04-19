classdef qcDMRAC < handle
    properties
        robot = quadcopter() % Actual quadcopter system
        model = quadcopter() % Ideal/model quadcopter system
        
        tSim
        xSim % states are x, dx, th, dth, i_motor
        xdotSim
        uSim
        
        linDMRAC = struct('Am', zeros(12),...
                          'Bm', zeros([12 4]),...
                          'Pm', zeros(12),...
                          'Qm', eye(12),...
                          'GammaR', eye(12),...
                          'GammaX', eye(12)*7.8,...
                          'Bdir', zeros([12 4]))
                
    end
    
    methods
        
        function obj = qcDMRAC(robot, model)
            if (nargin >= 1) 
                obj.robot = robot;
                if (nargin >= 2) 
                    obj.model = model;
                end
            end
            
            [A,B] = obj.model.getLinearization();
            K = obj.model.linContr.K;
            obj.linDMRAC.Am = A-B*K;
            obj.linDMRAC.Bm = B*K;
            
            obj.linDMRAC.Pm = obj.model.linContr.P;
            obj.linDMRAC.Qm = obj.model.linContr.Q;
            
            [Bt,Ba] = obj.robot.Bfun(1);
            obj.linDMRAC.Bdir = [zeros([3 4]);Bt;zeros([3 4]);Ba];
        end
        
        function setGammaR(self, gammaR)
            self.linDMRAC.gammaR = gammaR;
        end
        
        function setGammaX(self, gammaX)
            self.linDMRAC.gammaR = gammaX;
        end
        
        function [tSim, xSim, xdotSim, uSim] = runSim(self, tspan, x0, Kx0, Kr0, u, xref)
            sys = @(t,x) self.dynamics(t, x, u(t,x), xref(t));

            xvec0 = [x0;x0;Kx0(:);Kr0(:)];
            % Simulate the system.
            [tSim, xSim] = ode45(sys, tspan, xvec0);

            % Use simulation data to recover controls and diff eq.
            % Data is row-wise.
            num     = length(tSim);
            uSim    = zeros(num,4);
            xdotSim = zeros(size(xSim));
            
            for i = 1:num
                x = xSim(i,:)';
                t = tSim(i);
                uSim(i,:) = u(t, x)';
                xdotSim(i,:) = self.dynamics(t, x, u(t,x), xref(t))';
            end
            
            % Store variables in case needed for later.
            self.tSim = tSim;
            self.xSim = xSim;
            self.xdotSim = xdotSim;
            self.uSim = uSim;
        end
        
        function dx = dynamics(self, t, xvec, uvec, xref)
            dx = zeros([120 1]);
            
            xvec_x = xvec(1:12);
            xvec_m = xvec(13:24);
                        
            dx(1:12) = self.robot.dynamics(t,xvec,uvec);
            dx(13:24) = self.linDMRAC.Am*xvec_m + self.linDMRAC.Bm*xref;
            
            f = (xvec_x - xvec_m)'*self.linDMRAC.Pm*self.linDMRAC.Bdir;

            dKx = -self.linDMRAC.GammaX*xvec_x*f;
            dKr = -self.linDMRAC.GammaR*xref*f;
            
            dx(25:72) = dKx(:);
            dx(73:120) = dKr(:);
        end
        
        function u = DMRAC(self, t, xvec, xref)
            g = 9.81;
            xvec_x = xvec(1:12);
            Kx = reshape(xvec(25:72),[12 4]);
            Kr = reshape(xvec(73:120),[12 4]);

            u = (Kx.')*xvec_x + (Kr.')*xref + self.robot.param.m*g*[1;1;1;1]/(4*self.robot.param.b);
            disp(xvec(3));
        end
    end
end
