classdef TimeSeq < UAV.CmdSrcs.CmdSrc
    %TIMSEQ Time sequence of preset commands for UAV piloting
    %    Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        t_steps;     % Timesteps [1 x N] [s]
        acc_cmds;    % Global acceleration cmds [3 x N] [m/s^2]
        tz_cmds;     % Heading cmds [1 x N] [rad]
    end
    properties (Access = protected)
        i_cmd;   % Cmd index [cnts]
        n_cmd;   % Number of cmds [cnts]
    end
    
    methods (Access = public)
        function obj = TimeSeq(t_steps, acc_cmds, tz_cmds)
            %obj = TIMESEQ(t_steps, acc_cmds, tz_cmds)
            %    Construct command time sequence
            %    Inputs:
            %        t_steps = Timesteps [1 x N] [s]
            %        acc_cmds = Global acceleration cmds [3 x N] [m/s^2]
            %        tz_cmds = Heading cmds [1 x N] [rad]
            %    Timesteps must be evenly-spaced.
            obj.t_steps = t_steps;
            obj.acc_cmds = acc_cmds;
            obj.tz_cmds = tz_cmds;
            obj.i_cmd = 1;
            obj.n_cmd = length(t_steps);
        end
        
        function [cmd, t] = get_cmd(obj)
            %[cmd, t] = GET_CMD(obj) Get commands and time
            %   Outputs:
            %       cmd = UAV command [UAV.Cmd]
            %       t = Time [s]
            acc = obj.acc_cmds(:, obj.i_cmd);
            tz = obj.tz_cmds(obj.i_cmd);
            cmd = UAV.Cmd(acc, tz, 'Enabled');
            t = obj.t_steps(obj.i_cmd);
            obj.i_cmd = obj.i_cmd + 1;
        end
        
        function stop = get_stop(obj)
            %stop = GET_STOP(obj) Get stop flag [logical]
            stop = obj.i_cmd > obj.n_cmd;
        end
    end
end