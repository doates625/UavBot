classdef TimeSeq < UAV.CmdSrcs.CmdSrc
   %TIMSEQ Time sequence of preset commands for UAV piloting
   %    Author: Dan Oates (WPI Class of 2020)
   
   properties (SetAccess = protected)
       t_steps;     % Timesteps [1 x N] [s]
       acc_cmds;    % Global acceleration cmds [3 x N] [m/s^2]
       tz_cmds;     % Heading cmds [1 x N] [rad]
   end
   properties (Access = protected)
       f_sample;    % Sample rate [Hz]
       stop_flag;   % Stop flag [logical]
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
           obj.f_sample = 1 / (t_steps(2) - t_steps(1));
           obj.stop_flag = false;
       end
       
       function cmd = get_cmd(obj, t)
           %cmd = GET_CMD(obj, t) Get commands
           %   Inputs:
           %       t = Time [s]
           %   Outputs:
           %       cmd = UAV command [UAV.Cmd]
           
           % Check that t is in range
           if t < obj.t_steps(end)
               i = ceil(t * obj.f_sample);
           else
               i = length(obj.t_steps);
               obj.stop_flag = true;
           end
           
           % Construct cmd
           acc = obj.acc_cmds(:, i);
           tz = obj.tz_cmds(i);
           cmd = UAV.Cmd(acc, tz, 'Enabled');
       end
       
       function stop = get_stop(obj)
           %stop = GET_STOP(obj) Get stop flag [logical]
           stop = obj.stop_flag;
       end
   end
end