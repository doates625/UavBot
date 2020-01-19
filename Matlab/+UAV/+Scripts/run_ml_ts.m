function log = run_ml_ts()
%log = RUN_ML_TS() Run UAV matlab simulator with time sequence control
%   
%   Outputs:
%   - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('UAV.Interfaces.Sims.Matlab');
import('UAV.CmdSrcs.TimeSeq');
import('UAV.Scripts.run');

% Function
clc
uav = Matlab();
cmd_src = TimeSeq();
run_gui = true;
log = run(uav, cmd_src, run_gui);

end