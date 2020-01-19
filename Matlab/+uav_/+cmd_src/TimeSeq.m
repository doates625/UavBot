classdef TimeSeq < uav.cmd_src.CmdSrc
    %TIMSEQ Time sequence of preset commands for UAV piloting
    %   
    %    Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        time;       % Time of cmds [s]
        ang_pos;    % Angular position cmds [quat.Quat]
        thr_lin;    % Linear throttle cmds [0, 1]
    end
    properties (Access = protected)
        i_cmd;   % Cmd index [cnts]
        n_cmd;   % Number of cmds [cnts]
    end
    
    methods (Access = public)
        function obj = TimeSeq(model, params, time, ang_pos, thr_lin)
            %TIMESEQ Construct command time sequence
            %   
            %   obj = TIMESEQ(model, params, time, ang_pos, thr_lin)
            %       Construct custom sequence
            %       
            %       Inputs:
            %       - model = UAV model [uav.Model]
            %       - params = Flight params [uav.Params]
            %       - time = Time of cmds [s]
            %       - ang_pos = Angular position cmds [quat.Quat]
            %       - thr_lin = Linear throttle cmds [0, 1]
            %   
            %   obj = TIMESEQ(model) Construct default sequence with given model
            %   
            %   obj = TIMESEQ() Construct default sequence with default model
            
            % Imports
            import('uav.Params');
            import('uav.Model');
            
            % Superconstructor
            if nargin < 2, params = Params(); end
            if nargin < 1, model = Model(); end
            obj = obj@uav.cmd_src.CmdSrc(model, params);
            
            % Default commands
            if nargin < 5
                % Timesteps
                time_del = 1 / model.f_ctrl;
                time = 0:time_del:10;
                n = length(time);
                
                % Orientation cmds
                ang_z = zeros(1, n);
                ang_z(time >= 0) = time;
                ang_y = zeros(1, n);
                ang_y(time >= 0) = +deg2rad(20);
                ang_y(time >= 2) = -deg2rad(20);
                ang_y(time >= 4) = 0;
                ang_x = zeros(1, n);
                ang_x(time >= 4) = +deg2rad(20);
                ang_x(time >= 6) = -deg2rad(20);
                ang_x(time >= 8) = 0;
                ang_pos = obj.eul_to_quat(ang_z, ang_y, ang_x);
                
                % Throttle cmd
                thr_lin = obj.params.thr_max * ones(size(time));
            end
            
            % Copy args
            obj.time = time;
            obj.ang_pos = ang_pos;
            obj.thr_lin = thr_lin;
            obj.i_cmd = 1;
            obj.n_cmd = length(time);
        end
        
        function [cmd, time] = get_cmd(obj)
            %[cmd, time] = GET_CMD(obj) Get command and time
            %   
            %   Outputs:
            %   - cmd = UAV command [uav.Cmd]
            %   - time = Time [s]
            
            % Imports
            import('uav.state.Enum');
            import('uav.state.Cmd');
            
            % Form command
            ang_pos_ = obj.ang_pos(obj.i_cmd);
            thr_lin_ = obj.thr_lin(obj.i_cmd);
            enum = Enum.Enabled;
            cmd = Cmd(ang_pos_, thr_lin_, enum);
            
            % Form time
            time = obj.time(obj.i_cmd);
            
            % Increment counter
            obj.i_cmd = obj.i_cmd + 1;
        end
        
        function stop = get_stop(obj)
            %stop = GET_STOP(obj) Get stop flag [logical]
            stop = obj.i_cmd > obj.n_cmd;
        end
    end
end