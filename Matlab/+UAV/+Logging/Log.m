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
        uav;        % UAV interface [UAV.Interface]
        file_name;  % File name [char]
        states;     % States log [UAV.State.State]
        cmds;       % Commands log [UAV.State.Cmd]
        times;      % Time log [s]
        length_;    % Log length [cnts]
        trimmed;    % Trimmed flag [logical]
        comments;   % Comments [cell[char]
    end
    
    methods
        function obj = Log(uav, file_name)
            %LOG Create UAV flight log
            %   
            %   obj = LOG() Create empty UAV log
            %   obj = LOG(file_name) Load log from mat file
            %   obj = LOG('recent') Loads most recent log from mat file
            
            % Imports
            import('uav.state.State');
            import('uav.state.Cmd');
            import('uav.logging.Log');
            
            % File management
            if nargin < 2
                % Generate filename from time
                time = datetime(now, 'ConvertFrom', 'datenum');
                time.Format = 'MM-dd-HH-mm';
                obj.file_name = ['Log-', char(time)];
                
                % Pre-allocate log arrays
                n = obj.init_length;
                states(1, n) = State();
                cmds(1, n) = Cmd();
                obj.states = states;
                obj.cmds = cmds;
                obj.times = zeros(1, n);

                % Init fields
                obj.length_ = 0;
                obj.trimmed = false;
                obj.comments = {};
            else
                if strcmp(file_name, 'recent')
                    % Load most recent log
                    log_files = dir([obj.log_path, '*.mat']);
                    file_name = log_files(end).name;
                    file_name = file_name(1:end-4);
                    obj = Log(file_name);
                else
                    % Load from file
                    obj.file_name = file_name;
                    struct_ = load(obj.full_path());
                    obj = struct_.obj;
                end
            end
            
            % Copy UAV interface
            obj.uav = uav;
        end
       
        function update(obj, time)
            %UPDATE(obj, time) Add latest data to log
            %   Inputs:
            %   - time = Time [s]
            n = obj.length_ + 1;
            obj.states(n) = obj.uav.state;
            obj.cmds(n) = obj.uav.cmd;
            obj.times(n) = time;
            obj.length_ = n;
        end
        
        function obj = trim(obj)
            %obj = TRIM(obj) Trim empty pre-allocated space from log vectors
            if ~obj.trimmed
                n = obj.length_;
                obj.states = obj.states(1:n);
                obj.cmds = obj.cmds(1:n);
                obj.times = obj.times(1:n);
                obj.trimmed = true;
            end
        end
        
        function obj = crop(obj, t_min, t_max)
            %obj = CROP(obj, t_min, t_max)
            %   Crop log in time to range [t_min, t_max]
            
            % Compute endpoints
            i_min = find(obj.log_time > t_min, 1, 'first');
            i_max = find(obj.log_time > t_max, 1, 'first');
            
            % Trim logs to endpoints
            obj.states = obj.states(i_min:i_max);
            obj.cmds = obj.cmds(i_min:i_max);
            obj.times = obj.times(i_min:i_max);
            
            % Update log length
            obj.length_ = length(obj.time);
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
            %PLOT Plot log data with respect to time
            %   PLOT(obj, 'ang_pos') Plot orientation
            %   PLOT(obj, 'lin_acc') Plot linear acceleration
            %   PLOT(obj, 'thr_props') Plot prop throttles
            %   PLOT(obj, 'ang_ctrl') Plot angle control
            %   PLOT(obj, 'all') Plot all items above
            %   PLOT(obj) is shorthand for PLOT(obj, 'all')
            %   PLOT(obj, cell) Call plot for each name in cell
            
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
                    case 'lin_acc', obj.plot_lin_acc();
                    case 'thr_props', obj.plot_thr_props();
                    case 'ang_ctrl', obj.plot_ang_ctrl();
                    case 'all'
                        obj.plot_ang_pos();
                        obj.plot_lin_acc();
                        obj.plot_thr_props();
                        obj.plot_ang_ctrl();
                    otherwise, error('Invalid name: %s', name)
                end
            end
        end
        
        function save(obj)
            %SAVE(obj) Save log to mat file
            save(obj.full_path(), 'obj');
        end
    end
    
    methods (Access = protected)
        function path = full_path(obj)
            %path = FULL_PATH(obj)
            %   Full relative file path with '.mat' extension
            path = [obj.log_path, obj.file_name, '.mat'];
        end
        
        function plot_ang_pos(obj)
            %PLOT_ANG_POS(obj) Plot quat positions and setpoints
            obj.make_fig('Angular Position');
            axis_lbls = {'w', 'x', 'y', 'z'};
            ang_pos = [obj.states.ang_pos];
            ang_pos = [ang_pos.vector()];
            ang_cmd = [obj.cmds.ang_pos];
            ang_cmd = [ang_cmd.vector()];
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(sprintf('Quat-%s', axis_lbls{i}))
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.times, ang_cmd(i, :), 'k--')
                plot(obj.times, ang_pos(i, :)', 'b-')
                legend('Cmd', 'Val');
            end
        end
        
        function plot_lin_acc(obj)
            %PLOT_LIN_ACC(obj) Plot linear acceleration in new figure
            obj.make_fig('Linear Acceleration');
            vec_lbls = {'x', 'y', 'z'};
            lin_acc = [obj.states.lin_acc];
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Accel-' vec_lbls{i}])
                xlabel('Time [s]')
                ylabel('Accel [m/s^2]')
                plot(obj.times, lin_acc(i, :), 'b-')
            end
        end
        
        function plot_thr_props(obj)
            %PLOT_THR_PROPS(obj) Plot prop throttles in new figure
            obj.make_fig('Propeller Throttles');
            prop_lbls = {'++', '+-', '-+', '--'};
            thr_props = [obj.states.thr_props];
            for i = 1:4
                subplot(2, 2, i)
                hold on, grid on
                title(['Throttle [' prop_lbls{i} ']'])
                xlabel('Time [s]')
                ylabel('Throttle')
                plot(obj.times, thr_props(i, :), 'r-')
                ylim([0, 1])
            end
        end
        
        function plot_ang_ctrl(obj)
            %PLOT_ANG_CTRL(obj)
            %   Generate angle control plots in new figure
            
            % Imports
            import('uav.Model');
            
            % Compute angular errors and thrusts
            axis_lbls = {'x', 'y', 'z'};
            ang_pos = [obj.states.ang_pos];
            ang_cmd = [obj.cmds.ang_pos];
            ang_err = zeros(3, length(ang_pos));
            for i = 1:size(ang_err, 2)
                err = ang_cmd(i) \ ang_pos(i);
                err = err.vector();
                ang_err(:, i) = err(2:4);
            end
            thr_props = [obj.states.thr_props];
            thr_ang = Model.N_inv_ang * thr_props;
            
            % Generate plots
            obj.make_fig('Angular Position');
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(sprintf('Quat-%s Control', axis_lbls{i}))
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.times, ang_err(i, :), 'b-')
                plot(obj.times, thr_ang(i, :), 'r-')
                legend('Error', 'Throttle')
            end
        end
        
        function fig = make_fig(obj, name)
            %fig = MAKE_FIG(obj, window_title) Make new figure with given name
            name = [name, ' (', obj.file_name, ')'];
            fig = figure('Name', name);
        end
    end
end