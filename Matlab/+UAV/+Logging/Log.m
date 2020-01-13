classdef Log < handle
    %LOG Class for creating, processing, and saving UAV flight logs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Constant)
        log_path = 'Logs/';
    end
    
    properties (Access = protected, Constant)
        init_length = 1000;
    end
    
    properties (SetAccess = protected)
        file_name;      % File name [char]
        time;           % Time log [s]
        ang_pos;        % Orientation log [Quat]
        ang_vel;        % Local angular velocity log [rad/s]
        lin_acc;    	% Global linear acceleration log [m/s^2]
        thr_props;      % Prop throttles log [0, 1]
        ang_pos_cmd;    % Orientation cmd log [Quat]
        thr_lin_cmd;    % Linear throttle cmd [0, 1]
        log_length;     % Log length [cnts]
        trimmed;        % Trimmed flag [logical]
        comments;       % Comments [cell[char]
    end
    
    methods
        function obj = Log(file_name)
            %LOG Create UAV flight log
            %   obj = LOG() Create empty UAV log
            %   obj = LOG(file_name) Load log from mat file
            %   obj = LOG('recent') Loads most recent log from mat file
            if nargin < 1
                % Generate filename from time
                time = datetime(now, 'ConvertFrom', 'datenum');
                time.Format = 'MM-dd-HH-mm';
                obj.file_name = ['Log-', char(time)];
                
                % Pre-allocate log arrays
                n = obj.init_length;
                obj.time = zeros(1, n);
                ang_pos(1, n) = Quat();
                obj.ang_pos = ang_pos;
                obj.ang_vel = zeros(3, n);
                obj.lin_acc = zeros(3, n);
                obj.thr_props = zeros(4, n);
                ang_pos_cmd(1, n) = Quat();
                obj.ang_pos_cmd = ang_pos_cmd;
                obj.thr_lin_cmd = zeros(1, n);

                % Init fields
                obj.log_length = 0;
                obj.trimmed = false;
                obj.comments = {};
            else
                if strcmp(file_name, 'recent')
                    % Load most recent log
                    log_files = dir([obj.log_path, '*.mat']);
                    file_name = log_files(end).name;
                    file_name = file_name(1:end-4);
                    obj = UAV.Logging.Log(file_name);
                else
                    % Load from file
                    obj.file_name = file_name;
                    struct_ = load(obj.full_path());
                    obj = struct_.obj;
                end
            end
        end
       
        function update(obj, state, cmd, time)
            %UPDATE(obj, state, cmd, time) Add new data to log
            %   Inputs:
            %       state = UAV state [UAV.State.State]
            %       cmd = UAV command [UAV.State.Cmd]
            %       time = Time [s]
            n = obj.log_length + 1;
            obj.time(n) = time;
            obj.ang_pos(n) = state.ang_pos;
            obj.ang_vel(:, n) = state.ang_vel;
            obj.lin_acc(:, n) = state.lin_acc;
            obj.thr_props(:, n) = state.thr_props;
            obj.ang_pos_cmd(n) = cmd.ang_pos;
            obj.thr_lin_cmd(n) = cmd.thr_lin;
            obj.log_length = n;
        end
        
        function obj = trim(obj)
            %obj = TRIM(obj) Trims empty pre-allocated space from log vectors
            if ~obj.trimmed
                n = obj.log_length;
                obj.time = obj.time(1:n);
                obj.ang_pos = obj.ang_pos(1:n);
                obj.ang_vel = obj.ang_vel(:, 1:n);
                obj.lin_acc = obj.lin_acc(:, 1:n);
                obj.thr_props = obj.thr_props(:, 1:n);
                obj.ang_pos_cmd = obj.ang_pos_cmd(1:n);
                obj.thr_lin_cmd = obj.thr_lin_cmd(1:n);
                obj.trimmed = true;
            end
        end
        
        function obj = crop(obj, t_min, t_max)
            %obj = CROP(obj, t_min, t_max) Crops log in time to range [t_min, t_max]
            
            % Compute endpoints
            i_min = find(obj.log_time > t_min, 1, 'first');
            i_max = find(obj.log_time > t_max, 1, 'first');
            
            % Trim to endpoints
            obj.time = obj.time(i_min:i_max);
            obj.ang_pos = obj.ang_pos(i_min:i_max);
            obj.ang_vel = obj.ang_vel(:, i_min:i_max);
            obj.lin_acc = obj.lin_acc(:, i_min:i_max);
            obj.thr_props = obj.thr_props(:, i_min:i_max);
            obj.ang_pos_cmd = obj.ang_pos_cmd(i_min:i_max);
            obj.thr_lin_cmd = obj.thr_lin_cmd(i_min:i_max);
            
            % Update log length
            obj.log_length = length(obj.time);
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
            %   PLOT(obj, 'ang_ctrl') Plots angle control
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
                    case 'ang_ctrl', obj.plot_ang_ctrl();
                    case 'thr_props', obj.plot_thr_props();
                    case 'lin_acc', obj.plot_lin_acc();
                    case 'all'
                        obj.plot_ang_ctrl();
                        obj.plot_thr_props();
                        obj.plot_lin_acc();
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
        
        function plot_ang_ctrl(obj)
            %PLOT_ANG_CTRL(obj) Generates angle control plots in new figure
            obj.make_fig('Angle Control');
            axis_lbls = {'x', 'y', 'z'};
            ang_pos_ = obj.ang_pos.vector();
            ang_pos_cmd_ = obj.ang_pos_cmd.vector();
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(sprintf('Quat-%s Control', axis_lbls{i}))
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.time, ang_pos_cmd_(i+1, :), 'k--')
                plot(obj.time, ang_pos_(i+1, :)', 'b-')
                legend('Cmd', 'Val')
            end
        end
        
        function plot_thr_props(obj)
            %PLOT_THR_PROPS(obj) Plots prop throttles in new figure
            obj.make_fig('Propeller Throttles');
            prop_lbls = {'++', '+-', '-+', '--'};
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(['Throttle [' prop_lbls{i} ']'])
                xlabel('Time [s]')
                ylabel('Throttle [0, 1]')
                plot(obj.time, obj.thr_props(i, :), 'r-')
                ylim([0, 1])
            end
        end
        
        function plot_lin_acc(obj)
            %PLOT_LIN_ACC(obj) Plots linear acceleration in new figure
            obj.make_fig('Linear Acceleration');
            vec_lbls = {'x', 'y', 'z'};
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Accel-' vec_lbls{i}])
                xlabel('Time [s]')
                ylabel('Accel [m/s^2]')
                plot(obj.time, obj.lin_acc(i, :), 'b-')
            end
        end
        
        function fig = make_fig(obj, name)
            %fig = MAKE_FIG(obj, window_title) Makes new figure with given name
            name = [name, ' (', obj.file_name, ')'];
            fig = figure('Name', name);
        end
    end
end