function log = run_ml_xb()
%log = RUN_ML_XB() Run UAV matlab simulator with Xbox control
%   
%   Outputs:
%   - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('uav.interface.sim.Matlab');
import('uav.cmd_src.Xbox');
import('uav.scripts.run');

% Function
clc
uav = Matlab();
cmd_src = Xbox(uav);
run_gui = true;
save_log = false;
log = run(uav, cmd_src, run_gui, save_log);

end