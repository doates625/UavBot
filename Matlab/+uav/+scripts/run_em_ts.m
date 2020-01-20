function log = run_em_ts()
%log = RUN_EM_TS() Run UAV embedded simulator with time sequence control
%   
%   Outputs:
%  - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('uav.interface.sim.Embedded');
import('uav.cmd_src.TimeSeq');
import('uav.scripts.run');

% Function
clc
instrreset
uav = Embedded();
cmd_src = TimeSeq(uav);
run_gui = true;
save_log = false;
log = run(uav, cmd_src, run_gui, save_log);

end