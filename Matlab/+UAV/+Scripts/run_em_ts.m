function log = run_em_ts()
%log = RUN_EM_TS() Run UAV embedded simulator with time sequence control
%   Outputs:
%       log = Flight log object [UAV.Log]
%   Author: Dan Oates (WPI Class of 2020)

clc
instrreset
uav = UAV.Interfaces.Sims.Embedded();
cmd_src = UAV.CmdSrcs.TimeSeq();
run_gui = true;
log = UAV.Scripts.run(uav, cmd_src, run_gui);

end