function log = run_ml_xb()
%log = RUN_ML_XB() Run UAV matlab simulator with Xbox control
%   Outputs:
%       log = Flight log object [UAV.Log]
%   Author: Dan Oates (WPI Class of 2020)

clc
uav = UAV.Interfaces.Sims.Matlab();
cmd_src = UAV.CmdSrcs.Xbox();
run_gui = true;
log = UAV.Scripts.run(uav, cmd_src, run_gui);

end