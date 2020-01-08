function run_ml_ts()
%RUN_ML_TS() Run UAV matlab simulator with time sequence control

clc
uav = UAV.Interfaces.Sims.Matlab();
cmd_src = UAV.CmdSrcs.TimeSeq();
run_gui = false;
UAV.Scripts.run(uav, cmd_src, run_gui);

end