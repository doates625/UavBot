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
            %TIMESEQ Construct command time sequence
            %   
            %   obj = TIMESEQ(t_steps, acc_cmds, tz_cmds)
            %       Construct custom sequence
            %       Inputs:
            %           t_steps = Timesteps [1 x N] [s]
            %           acc_cmds = Global acceleration cmds [3 x N] [m/s^2]
            %           tz_cmds = Heading cmds [1 x N] [rad]
            %   
            %   obj = TIMESEQ() Construct default sequence
            %   
            %   Timesteps must be evenly-spaced.
            
            % Default args
            if nargin == 0
                % Timesteps
                f_sim = UAV.default_arg('f_sim');
                t_sim = 1 / f_sim;
                t_steps = 0:t_sim:10;
                N = length(t_steps);
                
                % Acceleration cmds
                acc_cmds = zeros(3, N);
                acc_cmds(1, t_steps >= 0) = +5;
                acc_cmds(1, t_steps >= 2) = -5;
                acc_cmds(1, t_steps >= 4) = 0;
                acc_cmds(2, t_steps >= 4) = +5;
                acc_cmds(2, t_steps >= 6) = -5;
                acc_cmds(2, t_steps >= 8) = 0;
                acc_cmds(3, t_steps >= 3) = +5;
                acc_cmds(3, t_steps >= 5) = -5;
                acc_cmds(3, t_steps >= 7) = 0;
                
                % Heading cmds
                tz_cmds = zeros(1, N);
                tz_cmds(t_steps >= 0) = t_steps;
                tz_cmds = wrap(tz_cmds, -pi, +pi);
            elseif nargin ~= 3
                error('Invalid nargin.')
            end
            
            % Copy args
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