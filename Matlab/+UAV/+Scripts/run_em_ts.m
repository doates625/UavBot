function log = run_em_ts()
%log = RUN_EM_TS() Run UAV embedded simulator with time sequence control
%   
%   Outputs:
%  - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('UAV.Interfaces.Sims.Embedded');
import('UAV.CmdSrcs.TimeSeq');
import('UAV.Scripts.run');

% Function
clc
instrreset
uav = Embedded();
cmd_src = TimeSeq();
run_gui = true;
log = run(uav, cmd_src, run_gui);

end