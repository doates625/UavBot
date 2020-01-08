function run_ml_xb()
%RUN_ML_XB() Run UAV matlab simulator with Xbox control

clc
uav = UAV.Interfaces.Sims.Matlab();
cmd_src = UAV.CmdSrcs.Xbox();
run_gui = true;
UAV.Scripts.run(uav, cmd_src, run_gui);

end