classdef quadcopter < handle
    properties
        param = struct('m',     0.5,...
                       'I',     diag([2.32e-3, 2.32e-3, 4e-3]),...
                       'L',     0.175,...
                       'Dt',    zeros(3), ...
                       'Dw',    zeros(3), ...
                       'b',     1, ...
                       'Km',    1.5e-9,...
                       'kF',    6.11e-8,...
                       'gamma', 1.5e-9/6.11e-8,...
                       'km',    20)
        
        tSim
        xSim % states are x, dx, th, dth, i_motor
        xdotSim
        uSim

        Bfun
        coords
        
    end
    
    methods
        %========================== quadcopter =========================
        %
        % Constructor for the quadcopter instance.
        %
        function obj = quadcopter(param)
             if (nargin >= 1) 
                  obj.param = param;
             end
             if isfield(obj.param,'Bfun')
               obj.Bfun = obj.param.Bfun;
             else
               obj.Bfun = @obj.Bplus;      % Defaults to plus configuration.
             end
             if isfield(obj.param,'coords')
               obj.coords = obj.param.coords;
             else
               obj.coords = obj.Coords_TaitBryanZXY();  % Defaults to ZXY.
             end
        end
        
        %============================ runSim ===========================
        %
        % Abstracted interface to run the simulation of the ducted
        % fan. Uses a format similar to odeXX functions for specifying the
        % initial value problem (IVP) with the addition of a control
        % function.
        %
        function [tSim, xSim, xdotSim, uSim] = runSim(self, tspan, x0, u)
            sys = @(t,x) self.dynamics(t, x, u);

            % Simulate the system.
            [tSim, xSim] = ode45(sys, tspan, x0);

            % Use simulation data to recover controls and diff eq.
            % Data is row-wise.
            num     = length(tSim);
            uSim    = zeros(num,4);
            xdotSim = zeros(size(xSim));
            
            for i = 1:num
                x = xSim(i,:)';
                t = tSim(i);
                uSim(i,:) = u(t, x)';
                xdotSim(i,:) = self.dynamics(t, x, u)';
            end
            
            % Store variables in case needed for later.
            self.tSim = tSim;
            self.xSim = xSim;
            self.xdotSim = xdotSim;
            self.uSim = uSim;
        end
        
        %=========================== dynamics ==========================
        %
        % Compute the dynamics of the system under the applied control.
        %
        % This particular version uses Euler angles for the orientation 
        % substate. If you'd like something else, like quaternions or rotation
        % matrices, then you'll have to implement yourself.
        %
        function dx = dynamics(self, t, xvec, uvec)
            % param
            m = self.param.m;
            I = self.param.I;
             
            % environment parameters
            g = 9.81;

            % Get orientation coordinate representation.
            oCoords = xvec(4:6);
                                  
            % Get translational velocity and body twist / angular rates.
            vT    = xvec(7:9);
            w     = xvec(10:12);

            % Get actuator input to state input matrices.
            [Bt, Ba] = self.Bfun(self.param.b);
            
            % Get applied u.
            u = uvec(t,xvec);
            
            % translation acceleration. 
            R  = self.coords.toR(oCoords);
            aT = [0;0;-g]  + (-self.param.Dt * vT + R*Bt*u)/m;
            
            % rotational dynamics
            wdot = I\( Ba*u - cross(w,I*w) - self.param.Dw*w );
            
            % Coordinate angle dynamics 
            OOYdot = self.coords.toW(oCoords) \ w;
            
            % stack derivatives
            dx = [vT; OOYdot; aT; wdot];
        end
        
        function dx = dynamics_symbolic(self,xvec,uvec)
            syms m;
            syms I [3 1];
            I = diag(I);
            syms Dt [3 3];
            syms Dw [3 3];
            syms b;
             
            % environment parameters
            syms g;

            % Get orientation coordinate representation.
            oCoords = xvec(4:6);
                                  
            % Get translational velocity and body twist / angular rates.
            vT    = xvec(7:9);
            w     = xvec(10:12);

            % Get actuator input to state input matrices.
            [Bt, Ba] = self.Bfun(b);
            
            % Get applied u.
            u = uvec;
            
            % translation acceleration. 
            R  = self.coords.toR(oCoords);
            aT = [0;0;-g]  + (-Dt * vT + R*Bt*u)/m;
            
            % rotational dynamics
            wdot = I\( Ba*u - cross(w,I*w) - Dw*w );
            
            % Coordinate angle dynamics 
            OOYdot = self.coords.toW(oCoords) \ w;
            
            % stack derivatives
            dx = [vT; OOYdot; aT; wdot];
        end

        %======================= linearController ======================
        %
        % Implements linear error feedback law with acceleration based
        % feed-forward term if argument provided.
        %
        % Assumes that xref packaged to be function of time. 
        % Assumes that aref packaged to be function of time. 
        %
        % To use, need to properly encapsulate within an anonymous function.
        %
        function u = linearController(self, t, xvec, xref, aref)
            
            % Compute any feedforward component.
            if (nargin > 5)
              % Handle feedforward, otherwise there is none.
            else
              uFF = 0; % Make proper dimension.
            end

            % Compute error-feedback.

            % Pack into control.
            
        end

        %===================== geometricController =====================
        %
        % Implements geometric error feedback law with acceleration based
        % feed-forward term if argument provided.
        %
        % Assumes that xref packaged to be function of time. 
        % Assumes that aref packaged to be function of time. 
        %
        % To use, need to properly encapsulate within an anonymous function.
        %
        function u = geometricController(self, t, xvec, xref, aref)
            
            % Compute any feedforward component.
            if (nargin > 5)
              % Handle feedforward, otherwise there is none.
            else
              uFF = 0; % Make proper dimension.
            end

            % Massage state and error signal into proper form.

            % Compute geometric error-feedback.

            % Pack into control.

            % Massage back if needed.
            
        end
        
    end

    methods (Static)
        %===================== Coords_TaitBryanZXY =====================
        %
        % Uses Tait-Bryan ZXY coordinates, which really means that the
        % rotation matrices applied are Ry(phi)Rx(theta)Rz(psi). The
        % ordering from left to right of the rotation matrices is flipped
        % relative to the coordinate rotation description.  This is because
        % the right-most matrix is applied first, and the leftmost last.
        % The coordinates are typically ordered based on a left to right
        % reading of the rotation matrices, which also disagrees with the
        % ZXY description. 
        %
        % Some people are not so picky about calling these Tait-Bryan
        % coordinates and will sometimes call them Euler coordinates.
        %
        function crep = Coords_TaitBryanZXY()

          crep.toR = @RTaitBryanZXY;
          crep.toW = @wTaitBryanZXY;

          
            %---------------------- RTaitBryanZXY ----------------------
            %
            %
            function R = RTaitBryanZXY(oCoords)
            
                phi   = oCoords(1);
                theta = oCoords(2);
                psi   = oCoords(3); 
                
                R = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), ...
                    -cos(phi)*sin(psi), ...
                    cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);   
                    cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), ...
                    cos(phi)*cos(psi), ...
                    sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
                    -cos(phi)*sin(theta), ...
                    sin(phi), ...
                    cos(phi)*cos(theta)];
                
            end

            %---------------------- wTaitBryanZXY ----------------------
            %
            %
            function W = wTaitBryanZXY(oCoords)

                phi   = oCoords(1);
                theta = oCoords(2);
                psi   = oCoords(3); 

                W = [cos(theta),  0 , -cos(phi)*sin(theta)  ; ...
                         0     ,  1 ,        sin(phi)       ; ...
                     sin(theta),  0 ,  cos(phi)*cos(theta)  ];

            end

        end

        %============================ Bplus ============================
        %
        % Get the control effectiveness matrices for the case that the 
        % quadcopter has a plus configuration.  Rotors 1 and 2 are along
        % the body x-axis (front to back), and rotors 3 and 4 are along the
        % body y-axis (right to left), where body x points forward and body
        % y point left.  As a consequence, body z points up.  This differs
        % from the aerospace convention of body z pointing down.
        %
        % The b input consists of the rotor speed to thrust gain.
        %
        function [Bt, Ba, Tr] = Bplus(b)

            Bt = b*[zeros(2,4); ones(1,4)];
            Ba = b*[0, 0, 1, -1; -1, 1, 0, 0; -1, -1, 1, 1];

            if (nargout == 3)
              Tr = transpose([1 0 0; -1 0 0; 0 1 0; 0 -1 0]);
            end

        end
          

    end

end

