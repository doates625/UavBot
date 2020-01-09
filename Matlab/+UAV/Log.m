classdef Log < handle
    %LOG Class for creating, processing, and saving UAV flight logs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        log_path = 'Logs/';
        init_length = 1000;
    end
    
    properties (SetAccess = protected)
        file_name;          % File name [char]
        log_time;           % Time log [s]
        log_ang_pos;        % Orientation log [w; x; y; z]
        log_ang_vel;        % Local angular velocity log [rad/s]
        log_lin_acc;    	% Global linear acceleration log [m/s^2]
        log_lin_acc_cmd;    % Global linear acceleration cmd log [m/s^2]
        log_ang_z;          % Heading log [rad]
        log_ang_z_cmd;      % Heading cmd log [rad]
        log_f_props;        % Propeller forces log [N]
        log_length;         % Log length [cnts]
        trimmed;            % Trimmed flag [logical]
        comments;           % Comments [cell[char]
    end
    
    methods
        function obj = Log(file_name)
            %LOG Create UAV flight log
            %   obj = LOG() Create empty UAV log
            %   obj = LOG(file_name) Load log from mat file
            if nargin < 1
                % Generate filename from time
                time = datetime(now, 'ConvertFrom', 'datenum');
                time.Format = 'MM-dd-HH-mm';
                obj.file_name = ['Log-', char(time)];
                
                % Pre-allocate log arrays
                n = obj.init_length;
                obj.log_time = zeros(1, n);
                obj.log_ang_pos = zeros(4, n);
                obj.log_ang_vel = zeros(3, n);
                obj.log_lin_acc = zeros(3, n);
                obj.log_lin_acc_cmd = zeros(3, n);
                obj.log_ang_z = zeros(1, n);
                obj.log_ang_z_cmd = zeros(1, n);
                obj.log_f_props = zeros(4, n);
                obj.log_length = 0;
                
                % Init fields
                obj.trimmed = false;
                obj.comments = {};
            else
                % Load from file
                obj.file_name = file_name;
                struct_ = load(obj.full_path());
                obj = struct_.obj;
            end
        end
       
        function update(obj, state, cmd, time)
            %UPDATE(obj, state, cmd, time) Add new data to log
            %   Inputs:
            %       state = UAV state [UAV.State.State]
            %       cmd = UAV command [UAV.State.Cmd]
            %       time = Time [s]
            n = obj.log_length + 1;
            obj.log_time(:, n) = time;
            obj.log_ang_pos(:, n) = state.ang_pos.vector();
            obj.log_ang_vel(:, n) = state.ang_vel;
            obj.log_lin_acc(:, n) = state.lin_acc;
            obj.log_lin_acc_cmd(:, n) = cmd.lin_acc;
            obj.log_ang_z(:, n) = state.ang_z;
            obj.log_ang_z_cmd(:, n) = cmd.ang_z;
            obj.log_f_props(:, n) = state.f_props;
            obj.log_length = n;
        end
        
        function obj = trim(obj)
            %obj = TRIM(obj) Trims empty pre-allocated space from log vectors
            if ~obj.trimmed
                n = obj.log_length;
                obj.log_time = obj.log_time(:, 1:n);
                obj.log_ang_pos = obj.log_ang_pos(:, 1:n);
                obj.log_ang_vel = obj.log_ang_vel(:, 1:n);
                obj.log_lin_acc = obj.log_lin_acc(:, 1:n);
                obj.log_lin_acc_cmd = obj.log_lin_acc_cmd(:, 1:n);
                obj.log_ang_z = obj.log_ang_z(:, 1:n);
                obj.log_ang_z_cmd = obj.log_ang_z_cmd(:, 1:n);
                obj.log_f_props = obj.log_f_props(:, 1:n);
                obj.trimmed = true;
            end
        end
        
        function obj = crop(obj, t_min, t_max)
            %obj = CROP(obj, t_min, t_max) Crops log in time to range [t_min, t_max]
            i_min = find(obj.log_time > t_min, 1, 'first');
            i_max = find(obj.log_time > t_max, 1, 'first');
            obj.log_time = obj.log_time(:, i_min:i_max);
            obj.log_ang_pos = obj.log_ang_pos(:, i_min:i_max);
            obj.log_ang_vel = obj.log_ang_vel(:, i_min:i_max);
            obj.log_lin_acc = obj.log_lin_acc(:, i_min:i_max);
            obj.log_lin_acc_cmd = obj.log_lin_acc_cmd(:, i_min:i_max);
            obj.log_ang_z = obj.log_ang_z(:, i_min:i_max);
            obj.log_ang_z_cmd = obj.log_ang_z_cmd(:, i_min:i_max);
            obj.log_f_props = obj.log_f_props(:, i_min:i_max);
            obj.log_length = length(obj.log_time);
        end
        
        function obj = cmt(obj, comment)
            %obj = CMT(obj, comment) Adds given comment [char] to log file
            obj.comments{end+1, 1} = comment;
        end
        
        function obj = cmtclr(obj)
            %obj = CMTCLR(obj) Clears all comments in log file
            obj.comments = {};
        end
        
        function plot(obj, name)
            %PLOT Plots log data with respect to time
            %   PLOT(obj, 'ang_pos') Plots orientation
            %   PLOT(obj, 'ang_vel') Plots angular velocity
            %   PLOT(obj, 'lin_acc') Plots linear acceleration
            %   PLOT(obj, 'ang_z') Plots heading
            %   PLOT(obj, 'f_props') Plots propeller forces
            %   PLOT(obj, 'all') Plots all items above
            %   PLOT(obj) is shorthand for PLOT(obj, 'all')
            %   PLOT(obj, cell) Calls plot for each name in cell
            
            % Default arg
            if nargin < 2
                name = 'all';
            end
            
            % Conditions
            if isa(name, 'cell')
                for i = 1:length(name)
                    obj.plot(name{i});
                end
            else
                switch name
                    case 'ang_pos', obj.plot_ang_pos();
                    case 'ang_vel', obj.plot_ang_vel();
                    case 'lin_acc', obj.plot_lin_acc();
                    case 'ang_z', obj.plot_ang_z();
                    case 'f_props', obj.plot_f_props();
                    case 'all'
                        obj.plot_ang_pos();
                        obj.plot_ang_vel();
                        obj.plot_lin_acc();
                        obj.plot_ang_z();
                        obj.plot_f_props();
                    otherwise, error('Invalid name: %s', name)
                end
            end
        end
        
        function save(obj)
            %SAVE(obj) Saves log to mat file
            save(obj.full_path(), 'obj');
        end
    end
    
    methods (Access = protected)
        function path = full_path(obj)
            %path = FULL_PATH(obj) Full relative file path with '.mat' extension
            path = [obj.log_path, obj.file_name, '.mat'];
        end
        
        function plot_ang_pos(obj)
            %PLOT_ANG_POS(obj) Plots angular position in new figure
            obj.make_fig('Orientation');
            quat_lbls = {'w', 'x', 'y', 'z'};
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(['Quat-' quat_lbls{i}])
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.log_time, obj.log_ang_pos(i, :), 'b-')
                ylim([-1, +1])
            end
        end
        
        function plot_ang_vel(obj)
            %PLOT_ANG_VEL(obj) Plots angular velocity in new figure
            obj.make_fig('Angular Velocity');
            vec_lbls = {'x', 'y', 'z'};
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Velocity-' vec_lbls{i}])
                xlabel('Time [s]')
                ylabel('Velocity [rad/s]')
                plot(obj.log_time, obj.log_ang_vel(i, :), 'b-')
            end
        end
        
        function plot_lin_acc(obj)
            %PLOT_LIN_ACC(obj) Plots linear acceleration in new figure
            obj.make_fig('Linear Acceleration Control');
            vec_lbls = {'x', 'y', 'z'};
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Accel-' vec_lbls{i}])
                xlabel('Time [s]')
                ylabel('Accel [m/s^2]')
                plot(obj.log_time, obj.log_lin_acc_cmd(i, :), 'k--')
                plot(obj.log_time, obj.log_lin_acc(i, :), 'b-')
                legend('Cmd', 'Val')
            end
        end
        
        function plot_ang_z(obj)
            %PLOT_ANG_Z(obj) Plot heading in new figure
            obj.make_fig('Heading Control');
            hold on, grid on
            title('Heading Control')
            xlabel('Time [s]')
            ylabel('Heading [rad]')
            plot(obj.log_time, obj.log_ang_z_cmd, 'k--')
            plot(obj.log_time, obj.log_ang_z, 'b-')
            ylim([-pi, +pi])
            legend('Cmd', 'Val')
        end
        
        function plot_f_props(obj)
            %PLOT_F_PROPS(obj) Plots prop forces in new figure
            obj.make_fig('Propeller Forces');
            model = UAV.Models.Phys();
            f_min = model.f_min;
            f_max = model.f_max;
            prop_lbls = {'++', '+-', '-+', '--'};
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(['Force [' prop_lbls{i} ']'])
                xlabel('Time [s]')
                ylabel('Force [N]')
                plot(obj.log_time, obj.log_f_props(i, :), 'r-')
                ylim([f_min, f_max])
            end
        end
        
        function fig = make_fig(obj, name)
            %fig = MAKE_FIG(obj, window_title) Makes new figure with given name
            name = [name, ' (', obj.file_name, ')'];
            fig = figure('Name', name);
        end
    end
end