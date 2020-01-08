function run_em_ts()
%RUN_EM_TS() Run UAV embedded simulator with time sequence control

clc
instrreset
uav = UAV.Interfaces.Sims.Embedded();
cmd_src = UAV.CmdSrcs.TimeSeq();
run_gui = true;
UAV.Scripts.run(uav, cmd_src, run_gui);

end