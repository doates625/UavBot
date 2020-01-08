function run_ml_ts()
%RUN_ML_TS() Run UAV matlab simulator with time sequence control

uav = UAV.Interfaces.Sims.Matlab();
cmd_src = UAV.CmdSrcs.TimeSeq();
UAV.Scripts.run(uav, cmd_src);

end