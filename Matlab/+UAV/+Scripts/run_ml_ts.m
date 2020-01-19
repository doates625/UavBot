function log = run_ml_ts()
%log = RUN_ML_TS() Run UAV matlab simulator with time sequence control
%   
%   Outputs:
%   - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('uav.interface.sim.Matlab');
import('uav.cmd_src.TimeSeq');
import('uav.scripts.run');

% Function
clc
uav = Matlab();
cmd_src = TimeSeq();
run_gui = true;
log = run(uav, cmd_src, run_gui);

end