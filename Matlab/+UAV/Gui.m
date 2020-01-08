classdef Gui < handle
    %GUI Classfor displaying and plotting UAV state in real-time
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        fig;            % Figure handle [Figure]
        plot_ang_pos;   % Orientation plot [FramePlot3D]
        plot_ang_vel;   % Angular velocity plot [VectorPlot3D]
        frame_cnt;      % Frame count [cnts]
        frame_rate;     % Frame rate estimate [Hz]
        timer;          % Timer object [Timer]
    end
    
    methods
        function obj = Gui(fig)
            %obj = GUI(fig) Create UAV GUI on given figure handle [Figure]
            %   If no handle is given, a new figure is created.
            
            % Get figure
            if nargin < 1, fig = figure; end
            obj.fig = fig;
            
            % Format figure
            figure(fig)
            fig.Units = 'normalized';
            fig.Position = [0.5005, 0.0380, 0.4990, 0.8833];
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
            obj.plot_ang_pos = FramePlot3D(1, 'r-', 'g-', 'b-');
            obj.plot_ang_vel = VectorPlot3D('k-');
            obj.plot_ang_pos.update(Quat());
            obj.plot_ang_vel.update(zeros(3, 1));
            
            % Make figure legend
            legend('x-hat', 'y-hat', 'z-hat', 'omega')
            
            % Frame rate tracking
            obj.frame_cnt = 0;
            obj.frame_rate = 0;
            obj.timer = Timer();
        end
        
        function update(obj, state, cmd, t)
            %UPDATE(obj, state, cmd, t) Update GUI with new state
            %   Inputs:
            %       state = UAV state [UAV.State]
            %       cmd = UAV command [UAV.Cmd]
            %       t = Time [s]
            
            % Start timing on first frame
            if obj.frame_cnt == 0
                obj.timer.tic();
            end
            
            % Update console
            clc
            fprintf('UAV GUI\n')
            fprintf('State: %s\n', state.state)
            fprintf('\nControls\n')
            fprintf('Accel-x [m/s^2]: Cmd = %+.2f, Act = %+.2f\n', cmd.acc(1), state.lin_acc(1));
            fprintf('Accel-y [m/s^2]: Cmd = %+.2f, Act = %+.2f\n', cmd.acc(2), state.lin_acc(2));
            fprintf('Accel-z [m/s^2]: Cmd = %+.2f, Act = %+.2f\n', cmd.acc(3), state.lin_acc(3));
            fprintf('Theta-z [rad/s]: Cmd = %+.2f, Act = %+.2f\n', cmd.tz, state.get_tz());
            fprintf('\nForces [N]:\n')
            fprintf('F++ = %.2f\n', state.f_prop(1));
            fprintf('F+- = %.2f\n', state.f_prop(2));
            fprintf('F-+ = %.2f\n', state.f_prop(3));
            fprintf('F-- = %.2f\n', state.f_prop(4));
            fprintf('\nFramerate:\n%.1f\n\n', obj.frame_rate)
            
            % Update plots
            figure(obj.fig);
            title(sprintf('UAV Pose (t = %.1f)', t))
            ang_pos = state.ang_pos;
            ang_vel = ang_pos.rotate(state.ang_vel);
            obj.plot_ang_pos.update(ang_pos);
            obj.plot_ang_vel.update(ang_vel);
            
            % Estimate frame rate
            obj.frame_cnt = obj.frame_cnt + 1;
            obj.frame_rate = obj.frame_cnt / obj.timer.toc();
        end
    end
end