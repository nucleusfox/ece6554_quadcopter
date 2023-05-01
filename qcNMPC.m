classdef qcNMPC < handle
    properties
        robot = quadcopter() % Actual quadcopter system
        model = quadcopter() % estimated/ideal model quadcopter system
        
        tSim
        xSim % states are x, dx, th, dth, i_motor
        xdotSim
        uSim
        
        nlmpcobj = nlmpc(12, 12, 4);
        nloptions = nlmpcmoveopt;
        lastMV = [0; 0; 0; 0];
        xHistory = (zeros(12))';
        rHistory = (zeros(12))';
        uHistory = [0; 0; 0; 0]';

        params = struct('Duration', 10, ...
                        'Ts', 0.1, ...
                        'ph', 18, ...
                        'ch', 2)
                
    end
    
    methods
        function setInitialX(self, x)
            self.xHistory = x';
        end

        function obj = qcNMPC(robot,model)
            if (nargin >= 1) 
                obj.robot = robot;
                if (nargin >= 2) 
                    obj.model = model;
                else
                    obj.model = robot;
                end
            end

            obj.nlmpcobj.Model.StateFcn = @(x,u) obj.model.dynamics(0,x,u);
            [A,B,xsym,usym] = obj.model.getJacobian();
            obj.nlmpcobj.Jacobian.StateFcn = matlabFunction(A,B,"vars",{xsym,usym});

            %rng(0)
            %validateFcns(obj.nlmpcobj,rand(12,1),rand(4,1));

            obj.nlmpcobj.Ts = obj.params.Ts;
            obj.nlmpcobj.PredictionHorizon = obj.params.ph;
            obj.nlmpcobj.ControlHorizon = obj.params.ch;
            obj.nlmpcobj.MV = struct( ...
                Min={0;0;0;0}, ...
                Max={10;10;10;10}, ...
                RateMin={-3;-3;-3;-3}, ...
                RateMax={3;3;3;3} ...
                );
            
            obj.nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
            obj.nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
            obj.nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
            
            g = 9.81;
            m = obj.model.param.m;
            b = obj.model.param.b;
            obj.nloptions.MVTarget = m*g*[1 1 1 1]/(4*b);
            obj.lastMV = obj.nloptions.MVTarget;
            
            obj.nlmpcobj.Ts = obj.params.Ts;
            obj.nlmpcobj.PredictionHorizon = obj.params.ph;
            obj.nlmpcobj.ControlHorizon = obj.params.ch;
        end
        
        function [tSim, xSim, xdotSim, uSim] = runSim(self, tspan, x0, xref)
            self.xHistory = x0';
            
            tSim = [];
            xSim = [];
            uSim = [];

            self.params.Duration = tspan(2);
            
            for k = 1:(self.params.Duration/self.params.Ts)
                % Compute control move with reference previewing
                xk = self.xHistory(k,:);
                [uk, yref] = self.uNMPC(k*self.params.Ts,xk,xref);
            
                % Store control move
                self.uHistory(k+1,:) = uk';
                self.lastMV = uk;
            
                % Simulate quadrotor for the next control interval (MVs = uk) 
                ODEFUN = @(t,xk) self.robot.dynamics(t,xk,uk);
                [TOUT,XOUT] = ode45(ODEFUN,[0 self.params.Ts], self.xHistory(k,:)');
            
                % Update quadrotor state
                self.xHistory(k+1,:) = XOUT(end,:);
    
                tSim = cat(1,tSim,TOUT(2:end) + (k-1)*self.params.Ts);
                xSim = cat(1,xSim,XOUT(2:end,:));
                uSim = cat(1,uSim,ones(length(TOUT)-1,1)*uk');
            end

            xdotSim = zeros(size(xSim));
            for i = 1:num
                x = xSim(i,:)';
                t = tSim(i);
                xdotSim(i,:) = self.robot.dynamics(t, x, uSim(i,:)', xref(t))';
            end

            self.tSim = tSim;
            self.xSim = xSim;
            self.xdotSim = xdotSim;
            self.uSim = uSim;
        end
                
        function [u, yref] = uNMPC(self, t, xvec, xref)
            % reference previewing
            tt = linspace(t, t+(self.params.ph-1)*self.params.Ts,self.params.ph);      
            yref = zeros([length(tt) 12]);
            for n = 1:length(tt)
                yref(n,:) = xref(tt(n))';
            end

            % compute control move
            u = nlmpcmove(self.nlmpcobj, xvec, self.lastMV, yref);
        end
    end
end