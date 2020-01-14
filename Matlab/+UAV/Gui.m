classdef Gui < handle
    %GUI Classfor displaying and plotting UAV state in real-time
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        uav;            % UAV interface [UAV.Interface]
        fig;            % Figure handle [Figure]
        plot_ang_pos;   % Orientation plot [FramePlot3D]
        plot_ang_cmd;   % Orientation cmd plot [FramePlot3D]
        plot_ang_vel;   % Angular velocity plot [VectorPlot3D]
        frame_cnt;      % Frame count [cnts]
        frame_rate;     % Frame rate estimate [Hz]
        timer;          % Timer object [Timer]
    end
    
    methods
        function obj = Gui(uav)
            %obj = GUI(uav)
            %   Create UAV GUI in new figure
            %   Inputs:
            %       uav = UAV interface [UAV.Interface]
            
            % Copy interface
            obj.uav = uav;
            
            % Format figure
            obj.fig = figure;
            obj.fig.Units = 'normalized';
            obj.fig.Position = [0.5005, 0.0380, 0.4990, 0.8833];
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
            obj.plot_ang_cmd = FramePlot3D(1, 'r--', 'g--', 'b--');
            obj.plot_ang_vel = VectorPlot3D('k-');
            obj.plot_ang_pos.update(Quat());
            obj.plot_ang_vel.update(zeros(3, 1));
            
            % Make figure legend
            legend('x-hat', 'y-hat', 'z-hat', 'x-hat cmd', 'y-hat cmd', 'z-hat cmd', 'ang-vel')
            
            % Frame rate tracking
            obj.frame_cnt = 0;
            obj.frame_rate = 0;
            obj.timer = Timer();
        end
        
        function update(obj, time)
            %UPDATE(obj, time) Update GUI
            %   Inputs:
            %       time = Time [s]
            
            % Start timing on first frame
            if obj.frame_cnt == 0
                obj.timer.tic();
            end
            
            % Copy state and cmd
            state = obj.uav.state;
            cmd = obj.uav.cmd;
            
            % Update printouts
            clc
            fprintf('UAV GUI\n')
            fprintf('State: %s\n\n', char(state.enum))
            
            % Orientation control print
            fprintf('Orientation Control\n')
            fprintf('Quat-w: Cmd = %+.2f, Act = %+.2f\n', cmd.ang_pos.w, state.ang_pos.w);
            fprintf('Quat-x: Cmd = %+.2f, Act = %+.2f\n', cmd.ang_pos.x, state.ang_pos.x);
            fprintf('Quat-y: Cmd = %+.2f, Act = %+.2f\n', cmd.ang_pos.y, state.ang_pos.y);
            fprintf('Quat-z: Cmd = %+.2f, Act = %+.2f\n', cmd.ang_pos.z, state.ang_pos.z);
            
            % Prop throttles print
            fprintf('\nProp Throttles:\n')
            fprintf('T++ = %.2f\n', state.thr_props(1));
            fprintf('T+- = %.2f\n', state.thr_props(2));
            fprintf('T-+ = %.2f\n', state.thr_props(3));
            fprintf('T-- = %.2f\n', state.thr_props(4));
            
            % Framerate print
            fprintf('\nFramerate:\n%.1f\n\n', obj.frame_rate)
            
            % Update plots
            figure(obj.fig);
            title(sprintf('UAV Pose (t = %.1f)', time))
            ang_pos = state.ang_pos;
            ang_cmd = cmd.ang_pos;
            ang_vel = ang_pos.rotate(state.ang_vel);
            obj.plot_ang_pos.update(ang_pos);
            obj.plot_ang_cmd.update(ang_cmd);
            obj.plot_ang_vel.update(ang_vel);
            
            % Estimate frame rate
            obj.frame_cnt = obj.frame_cnt + 1;
            obj.frame_rate = obj.frame_cnt / obj.timer.toc();
        end
    end
end