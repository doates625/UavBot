classdef UavPlot
    %UAVPLOT Class for plotting UAV state in real-time
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        fig;    % Figure handle [Figure]
        plot_q; % Orientation plot [FramePlot3D]
        plot_w; % Angular velocity plot [VectorPlot3D]
    end
    
    methods
        function obj = UavPlot(fig)
            %obj = UAVPLOT(fig) Creates UAV plot
            %   Inputs:
            %       fig = Figure handle
            %   If no handle is given, a new figure is created.
            
            % Get figure
            if nargin < 1, fig = figure; end
            obj.fig = fig;
            
            % Format figure
            figure(fig)
            clf, hold on, grid on
            title('UAV Pose')
            xlabel('x')
            ylabel('y')
            zlabel('z')
            axis([-1, 1, -1, 1, -1, 1])
            view(-20, +35)
            camproj perspective
            axis square
            
            % Create plot objects
            obj.plot_q = FramePlot3D(1, 'r-', 'g-', 'b-');
            obj.plot_w = VectorPlot3D('k-');
            obj.plot_q.update(Quat());
            obj.plot_w.update(zeros(3, 1));
            
            % Make figure legend
            legend('x-hat', 'y-hat', 'z-hat', 'omega')
        end
        
        function update(obj, q, w, t)
            %UPDATE(obj, q, w) Update plot with new state
            %   Inputs:
            %       q = Orientation [quat]
            %       w = Local angular velocity [rad/s]
            %       t = Time [s]
            %   If t is given, it is added to the title
            figure(obj.fig);
            if nargin < 4
                title('UAV Pose')
            else
                title(sprintf('UAV Pose (t = %.2f)', t))
            end
            w = q.rotate(w);
            obj.plot_q.update(q);
            obj.plot_w.update(w);
        end
    end
end

