classdef quadcopterAnimator < handle
    
    properties
        X
        U
        lastPlot
        vecMovie
        param
        fig = NaN;

        drone;
    end
    
    methods
        function obj = quadcopterAnimator(drone, fig)
             obj.drone  = drone;

             if (nargin > 1)
                  obj.fig = fig;
             end
        end
                
        function animateInput(self, t, X, U, factor)
            if ~exist('factor','var')
                factor = 1;
            end
            if(factor<1)
                factor = 1;
                warning('factor has been clamped to 1. factor must be (factor>=1)!')
            end
            if ~isgraphics(self.fig)
                self.fig = figure();
            end
          
            % Use simulation limits to establish plot window size.
            % Keep square due to axis equal.
            wrad = 0.5;
            winx = [min(X(:,1))-wrad, max(X(:,1))+wrad];
            winy = [min(X(:,2))-wrad, max(X(:,2))+wrad];
            winz = [min(X(:,3))-wrad, max(X(:,3))+wrad];
                        
            % Get max and min over both control signals.
            % Add buffer above and below.
            urad = 1;
            y_ulim = [min(U(:))-urad, max(U(:))+urad];
            
            % Check if have drawing from previous calls.
            % Setup plot times.
            N = size(X,1);
            
            P = [];
            Pt = [];

            subplot(1,2,1);
                axis equal
                xlim(winx)
                ylim(winy)
                zlim(winz)
                view(3)

            % Plot the entire control signal.  Add vertical line that will
            % sweep across the graph.
            subplot(1,2,2);
                xlabel('Time (sec)')
                ylabel('$u(t)$')
                plot(t, U);
                Pl = line(t([1 1]), transpose(y_ulim),'LineStyle',':');
                xlim([t(1), t(end)]);
                ylim(y_ulim);

            hold on
            tmp = 1:factor:N;
            if tmp(end) ~= N
                tmp = [tmp, N];
            end

            % Iterate through plot times and draw quadcopter.
            k = 1;
            for i = tmp
                subplot(1,2,1);
                if (isempty(Pt))
                  Pt = line(X(k:i,1), X(k:i,2), X(k:i,3), 'LineStyle','-.', ...
                             'color', '[0.3010, 0.7450, 0.9330]',...
                             'marker','o','markersize', 0.2);
                  hold on;
                else
                  set(Pt,'XData',X(1:i,1),'YData',X(1:i,2),'ZData',X(1:i,3));
                  hold off;
                end
                set(Pl,'XData', t([i i]));

                P = self.snapshot(X(i,:)', P);  
                title("Time = " + sprintf('%.2f',t(i)) + " sec")
                drawnow
                vec(i) = getframe(self.fig);
                k = i;
            end
            self.vecMovie = vec;
        end

        function animate(self, t, X, factor)
            if ~exist('factor','var')
                factor = 1;
            end
            if(factor<1)
                factor = 1;
                warning('factor has been clamped to 1. factor must be (factor>=1)!')
            end
            if ~isgraphics(self.fig)
                self.fig = figure();
            end
          
            % Use simulation limits to establish plot window size.
            % Keep square due to axis equal.
            wrad = 1;
            winx = [min(X(:,1))-wrad, max(X(:,1))+wrad];
            winy = [min(X(:,2))-wrad, max(X(:,2))+wrad];
            winz = [min(X(:,3))-wrad, max(X(:,3))+wrad];
                        
            % Check if have drawing from previous calls.
            % Setup plot times.
            N = size(X,1);
            
            P = [];
            Pt = [];

            figure(self.fig);
                axis equal
                xlim(winx)
                ylim(winy)
                zlim(winz)

            tmp = 1:factor:N;
            if tmp(end) ~= N
                tmp = [tmp, N];
            end
            % Iterate through plot times and draw quadcopter.
            k = 1;
            view(3)
            for i = tmp
                if (isempty(Pt))
                  Pt = line(X(k:i,1), X(k:i,2), X(k:i,3), 'LineStyle', '-.', ...
                             'color', '[0.3010, 0.7450, 0.9330]',...
                             'marker','o','markersize', 0.2);
                else
                  set(Pt,'XData',X(1:i,1),'YData',X(1:i,2),'ZData',X(1:i,3));
                end

                P = self.snapshot(X(i,:)', P);
                title("Time = " + sprintf('%.2f',t(i)) + " sec")
                drawnow

                vec(i) = getframe(self.fig);
                k = i;
            end
            self.vecMovie = vec;
        end
        
        %=========================== snapshot ==========================
        %
        % Create a snapshot of the quadcopter in the current axis.
        %
        function P = snapshot(self, X, P)
            if (nargin < 3)
               P = [];
            end

            [Bt, Ba, Tr] = self.drone.Bfun(self.drone.param.b);
            R = self.drone.coords.toR(X(4:6));

            axSize = 0.5;
            L = self.drone.param.L;
            
            pWR = X(1:3);
            u   = [pWR, pWR];
            
            bx = R*[0,axSize; 0,0; 0,0] + u;
            by = R*[0,0; 0,axSize; 0,0] + u;
            bz = R*[0,0; 0,0; 0,axSize] + u;

            l1 = R*L*Tr(:,1:2) + u;
            l2 = R*L*Tr(:,3:4) + u;
            
            if (isempty(P))
                % Plot body axes in world frame.
                P(1) = line(bx(1,:),bx(2,:),bx(3,:), ...
                                                'linewidth',3,'color','green');
                P(2) = line(by(1,:),by(2,:),by(3,:), ...
                                                'linewidth',3,'color','red');
                P(3) = line(bz(1,:),bz(2,:),bz(3,:), ...
                                                'linewidth',3,'color','blue');
            
                % Plot quadrotor frame
                P(4) = line(l1(1,:),l1(2,:),l1(3,:), ...
                                                'linewidth',3,'color','black');
                P(5) = line(l2(1,:),l2(2,:),l2(3,:), ...
                                                'linewidth',3,'color','black');
            
                % qaudrotor propellers
                P(6) = line(l1(1,1),l1(2,1),l1(3,1), ...
                           'Marker','o','MarkerSize',L*20, ...
                           'MarkerFaceColor','#7E2F8E', 'color','#7E2F8E');
                P(7) = line(l2(1,1),l2(2,1),l2(3,1), ...
                           'Marker','o','MarkerSize',L*20, ...
                           'MarkerFaceColor','#7E2F8E', 'color','#7E2F8E');
                P(8) = line(l1(1,2),l1(2,2),l1(3,2), ...
                           'Marker','o','MarkerSize',L*20, ...
                           'MarkerFaceColor','#7E2F8E', 'color','#7E2F8E');
                P(9) = line(l2(1,2),l2(2,2),l2(3,2), ...
                           'Marker','o','MarkerSize',L*20, ...
                           'MarkerFaceColor','#7E2F8E', 'color','#7E2F8E');
            else
              set(P(1),'XData',bx(1,:),'YData',bx(2,:),'ZData',bx(3,:));
              set(P(2),'XData',by(1,:),'YData',by(2,:),'ZData',by(3,:));
              set(P(3),'XData',bz(1,:),'YData',bz(2,:),'ZData',bz(3,:));

              set(P(4),'XData',l1(1,:),'YData',l1(2,:),'ZData',l1(3,:));
              set(P(5),'XData',l2(1,:),'YData',l2(2,:),'ZData',l2(3,:));

              set(P(6),'XData',l1(1,1),'YData',l1(2,1),'ZData',l1(3,1));
              set(P(7),'XData',l2(1,1),'YData',l2(2,1),'ZData',l2(3,1));
              set(P(8),'XData',l1(1,2),'YData',l1(2,2),'ZData',l1(3,2));
              set(P(9),'XData',l2(1,2),'YData',l2(2,2),'ZData',l2(3,2));
            end
        end
        
        function export_movie(self, txt, fr)
            num = length(self.vecMovie);
            
            for i = 1:(num+10)
                if i <= 10
                    vec(i) = self.vecMovie(1);
                else
                    vec(i) = self.vecMovie(i-10);
                end
            end
%             myWriter = VideoWriter(txt, 'MPEG-4');
            myWriter = VideoWriter(txt);
            myWriter.FrameRate = fr;
            
            open(myWriter);
            writeVideo(myWriter, vec);
            close(myWriter)
        end
    end
end

