classdef TimeSeq < UAV.CmdSrcs.CmdSrc
    %TIMSEQ Time sequence of preset commands for UAV piloting
    %    Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        time_cmds;      % Time of cmds [s]
        ang_pos_cmds;   % Angular position cmds [Quat]
        thr_lin_cmds;   % Linear throttle cmds [0, 1]
    end
    properties (Access = protected)
        i_cmd;   % Cmd index [cnts]
        n_cmd;   % Number of cmds [cnts]
    end
    
    methods (Access = public)
        function obj = TimeSeq(model, params, time_cmds, ang_pos_cmds, thr_lin_cmds)
            %TIMESEQ Construct command time sequence
            %   obj = TIMESEQ(model, params, time_cmds, ang_pos_cmds, thr_lin_cmds)
            %       Construct custom sequence
            %       Inputs:
            %           model = UAV model [UAV.Model]
            %           params = Flight params [UAV.Params]
            %           time_cmds = Time of cmds [s]
            %           ang_pos_cmds = Angular position cmds [Quat]
            %           thr_lin_cmds = Linear throttle cmds [0, 1]
            %   obj = TIMESEQ(model) Construct default sequence with given model
            %   obj = TIMESEQ() Construct default sequence with default model
            
            % Superconstructor
            if nargin < 2, params = UAV.Params(); end
            if nargin < 1, model = UAV.Model(); end
            obj = obj@UAV.CmdSrcs.CmdSrc(model, params);
            
            % Default commands
            if nargin < 5
                % Timesteps
                time_del = 1 / model.f_ctrl;
                time_cmds = 0:time_del:10;
                n = length(time_cmds);
                
                % Orientation cmds
                ang_z = zeros(1, n);
                ang_z(time_cmds >= 0) = time_cmds;
                ang_y = zeros(1, n);
                ang_y(time_cmds >= 0) = +deg2rad(20);
                ang_y(time_cmds >= 2) = -deg2rad(20);
                ang_y(time_cmds >= 4) = 0;
                ang_x = zeros(1, n);
                ang_x(time_cmds >= 4) = +deg2rad(20);
                ang_x(time_cmds >= 6) = -deg2rad(20);
                ang_x(time_cmds >= 8) = 0;
                ang_pos_cmds = obj.eul_to_quat(ang_z, ang_y, ang_x);
                
                % Throttle cmd
                thr_lin_cmds = obj.params.thr_max * ones(size(time_cmds));
            end
            
            % Copy args
            obj.time_cmds = time_cmds;
            obj.ang_pos_cmds = ang_pos_cmds;
            obj.thr_lin_cmds = thr_lin_cmds;
            obj.i_cmd = 1;
            obj.n_cmd = length(time_cmds);
        end
        
        function [cmd, time] = get_cmd(obj)
            %[cmd, time] = GET_CMD(obj) Get command and time
            %   Outputs:
            %       cmd = UAV command [UAV.Cmd]
            %       time = Time [s]
            
            % Form command
            ang_pos = obj.ang_pos_cmds(obj.i_cmd);
            thr_lin = obj.thr_lin_cmds(obj.i_cmd);
            enum = UAV.State.Enum.Enabled;
            cmd = UAV.State.Cmd(ang_pos, thr_lin, enum);
            
            % Form time
            time = obj.time_cmds(obj.i_cmd);
            
            % Increment counter
            obj.i_cmd = obj.i_cmd + 1;
        end
        
        function stop = get_stop(obj)
            %stop = GET_STOP(obj) Get stop flag [logical]
            stop = obj.i_cmd > obj.n_cmd;
        end
    end
end