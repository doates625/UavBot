function log = run_ml_xb()
%log = RUN_ML_XB() Run UAV matlab simulator with Xbox control
%   
%   Outputs:
%   - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('UAV.Interfaces.Sims.Matlab');
import('UAV.CmdSrcs.Xbox');
import('UAV.Scripts.run');

% Function
clc
uav = Matlab();
cmd_src = Xbox();
run_gui = true;
log = run(uav, cmd_src, run_gui);

end