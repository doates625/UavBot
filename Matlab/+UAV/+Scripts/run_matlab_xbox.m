function run_matlab_xbox()
%RUN_MATLAB_XBOX() Run UAV matlab simulator with Xbox control

% Construct run args
import('UAV.default_arg');
f_sim = 40;
phys_model = default_arg('phys_model');
ctrl_model = default_arg('ctrl_model');
uav = UAV.Interfaces.Sims.Matlab(phys_model, ctrl_model, f_sim);
cmd_src = UAV.CmdSrcs.Xbox();

% Call run function
UAV.Scripts.run(uav, cmd_src);

end