classdef Log < handle
    %LOG Class for logging data from UAV runs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        axis_names = {'x', 'y', 'z'};
        prop_names = {'++', '+-', '-+', '--'};
        quat_axes = {'w', 'x', 'y', 'z'};
    end
    
    properties (SetAccess = protected)
        t;          % Timesteps [s]
        ang_pos;    % Orientations [Quat]
        ang_vel;    % Local angular velocities [rad/s]
        lin_acc;    % Global linear accelerations [m/s^2]
        f_prop;     % Prop forces [N]
        tz;         % Headings [rad]
        acc_cmd;    % Global acceleration cmds [m/s^2]
        tz_cmd;     % Heading cmds [rad]
        n;          % Log length [cnts]
        trimmed;    % Trimmed flag [logical]
    end
    
    methods
        function obj = Log()
            %obj = LOG() Create empty UAV log
            n = 1000;
            obj.t = zeros(1, n);
            obj.ang_pos = zeros(4, n);
            obj.ang_vel = zeros(3, n);
            obj.lin_acc = zeros(3, n);
            obj.f_prop = zeros(4, n);
            obj.tz = zeros(1, n);
            obj.acc_cmd = zeros(3, n);
            obj.tz_cmd = zeros(1, n);
            obj.n = 0;
            obj.trimmed = false;
        end
       
        function update(obj, state, cmd, t)
            %UPDATE(obj, state, cmd, t) Add new data to log
            %   Inputs:
            %       state = UAV state [UAV.State]
            %       cmd = UAV command [UAV.Cmd]
            %       t = Time [s]
            obj.n = obj.n + 1;
            obj.t(obj.n) = t;
            obj.ang_pos(:, obj.n) = state.ang_pos.vector();
            obj.ang_vel(:, obj.n) = state.ang_vel;
            obj.lin_acc(:, obj.n) = state.lin_acc;
            obj.f_prop(:, obj.n) = state.f_prop;
            obj.tz(obj.n) = state.get_tz();
            obj.acc_cmd(:, obj.n) = cmd.acc;
            obj.tz_cmd(obj.n) = cmd.tz;
        end
        
        function plot(obj)
            %PLOT(obj) Plots log data
            
            % Trim log vectors
            obj.trim();
            
            % Acceleration
            figure;
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Accel-' obj.axis_names{i} ' Control'])
                xlabel('Time [s]')
                ylabel('Accel [m/s^2]')
                plot(obj.t, obj.acc_cmd(i, :), 'k--')
                plot(obj.t, obj.lin_acc(i, :), 'b-')
                legend('Setpt', 'Value')
            end
            
            % Heading
            figure;
            hold on, grid on
            title('Heading Control')
            xlabel('Time [s]')
            ylabel('Heading [rad]')
            plot(obj.t, obj.tz_cmd, 'k--')
            plot(obj.t, obj.tz, 'b-')
            ylim([-pi, +pi])
            legend('Setpt', 'Value')
            
            % Angular position
            figure;
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(['Quat-' obj.quat_axes{i}])
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.t, obj.ang_pos(i, :), 'b-')
            end
            
            % Angular velocity
            figure;
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Omega-' obj.axis_names{i}])
                xlabel('Time [s]')
                ylabel('Velocity [rad/s]')
                plot(obj.t, obj.ang_vel(i, :), 'b-')
            end
            
            % Prop forces
            figure;
            model = UAV.Models.Phys();
            f_min = model.f_min;
            f_max = model.f_max;
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(['Force [' obj.prop_names{i} ']'])
                xlabel('Time [s]')
                ylabel('Force [N]')
                plot(obj.t, obj.f_prop(i, :), 'r-')
                ylim([f_min, f_max])
            end
        end
    end
    
    methods (Access = protected)
        function trim(obj)
            %TRIM(obj) Trims log vectors if not already done
            if ~obj.trimmed
                obj.t = obj.t(1:obj.n);
                obj.ang_pos = obj.ang_pos(:, 1:obj.n);
                obj.ang_vel = obj.ang_vel(:, 1:obj.n);
                obj.lin_acc = obj.lin_acc(:, 1:obj.n);
                obj.f_prop = obj.f_prop(:, 1:obj.n);
                obj.tz = obj.tz(1:obj.n);
                obj.acc_cmd = obj.acc_cmd(:, 1:obj.n);
                obj.tz_cmd = obj.tz_cmd(1:obj.n);
                obj.trimmed = true;
            end
        end
    end
end