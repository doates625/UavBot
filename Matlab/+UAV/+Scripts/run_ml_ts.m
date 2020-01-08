function log = run_ml_ts()
%log = RUN_ML_TS() Run UAV matlab simulator with time sequence control
%   Outputs:
%       log = Flight log object [UAV.Log]
%   Author: Dan Oates (WPI Class of 2020)

clc
uav = UAV.Interfaces.Sims.Matlab();
cmd_src = UAV.CmdSrcs.TimeSeq();
run_gui = false;
log = UAV.Scripts.run(uav, cmd_src, run_gui);

end