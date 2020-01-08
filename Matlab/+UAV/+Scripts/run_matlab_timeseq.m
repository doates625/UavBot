function run_matlab_timeseq()
%RUN_MATLAB_XBOX() Run UAV matlab simulator with time sequence control

% Construct run args
import('UAV.default_arg');

% Matlab simulator
f_sim = 50;
phys_model = default_arg('phys_model');
ctrl_model = default_arg('ctrl_model');
uav = UAV.Interfaces.Sims.Matlab(phys_model, ctrl_model, f_sim);

% Command sequence
t_sim = 1 / f_sim;
t_steps = 0:t_sim:10;
N = length(t_steps);
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
tz_cmds = t_steps;
tz_cmds = wrap(tz_cmds, -pi, +pi);
cmd_src = UAV.CmdSrcs.TimeSeq(t_steps, acc_cmds, tz_cmds);

% Call run function
UAV.Scripts.run(uav, cmd_src);

end