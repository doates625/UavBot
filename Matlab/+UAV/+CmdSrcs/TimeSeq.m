classdef TimeSeq < UAV.CmdSrcs.CmdSrc
    %TIMSEQ Time sequence of preset commands for UAV piloting
    %    Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        time_cmds;      % Time of cmds [s]
        lin_acc_cmds;   % Global linear acceleration cmds [m/s^2]
        ang_z_cmds;     % Heading cmds [rad]
    end
    properties (Access = protected)
        i_cmd;   % Cmd index [cnts]
        n_cmd;   % Number of cmds [cnts]
    end
    
    methods (Access = public)
        function obj = TimeSeq(time_cmds, lin_acc_cmds, ang_z_cmds)
            %TIMESEQ Construct command time sequence
            %   obj = TIMESEQ(time_cmds, lin_acc_cmds, ang_z_cmds)
            %       Construct custom sequence
            %       Inputs:
            %           time_cmds = Time of cmds [s]
            %           lin_acc_cmds = Global linear acceleration cmds [m/s^2]
            %           ang_z_cmds = Heading cmds [rad]
            %   obj = TIMESEQ() Construct default sequence
            
            % Default args
            if nargin == 0
                % Timesteps
                f_step = 50.0;
                t_step = 1 / f_step;
                time_cmds = 0:t_step:10;
                n = length(time_cmds);
                
                % Acceleration cmds
                lin_acc_cmds = zeros(3, n);
                lin_acc_cmds(1, time_cmds >= 0) = +5;
                lin_acc_cmds(1, time_cmds >= 2) = -5;
                lin_acc_cmds(1, time_cmds >= 4) = 0;
                lin_acc_cmds(2, time_cmds >= 4) = +5;
                lin_acc_cmds(2, time_cmds >= 6) = -5;
                lin_acc_cmds(2, time_cmds >= 8) = 0;
                lin_acc_cmds(3, time_cmds >= 3) = +5;
                lin_acc_cmds(3, time_cmds >= 5) = -5;
                lin_acc_cmds(3, time_cmds >= 7) = 0;
                
                % Heading cmds
                ang_z_cmds = zeros(1, n);
                ang_z_cmds(time_cmds >= 0) = time_cmds;
                ang_z_cmds = wrap(ang_z_cmds, -pi, +pi);
            elseif nargin ~= 3
                error('Invalid nargin.')
            end
            
            % Copy args
            obj.time_cmds = time_cmds;
            obj.lin_acc_cmds = lin_acc_cmds;
            obj.ang_z_cmds = ang_z_cmds;
            obj.i_cmd = 1;
            obj.n_cmd = length(time_cmds);
        end
        
        function [cmd, time] = get_cmd(obj)
            %[cmd, time] = GET_CMD(obj) Get command and time
            %   Outputs:
            %       cmd = UAV command [UAV.Cmd]
            %       time = Time [s]
            lin_acc = obj.lin_acc_cmds(:, obj.i_cmd);
            ang_z = obj.ang_z_cmds(obj.i_cmd);
            cmd = UAV.State.Cmd(lin_acc, ang_z, UAV.State.Enum.Enabled);
            time = obj.time_cmds(obj.i_cmd);
            obj.i_cmd = obj.i_cmd + 1;
        end
        
        function stop = get_stop(obj)
            %stop = GET_STOP(obj) Get stop flag [logical]
            stop = obj.i_cmd > obj.n_cmd;
        end
    end
end