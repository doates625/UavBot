function run_ml_xb()
%RUN_ML_XB() Run UAV matlab simulator with Xbox control

uav = UAV.Interfaces.Sims.Matlab();
cmd_src = UAV.CmdSrcs.Xbox();
UAV.Scripts.run(uav, cmd_src);

end