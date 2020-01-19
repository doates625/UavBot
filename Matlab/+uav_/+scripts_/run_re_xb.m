function log = run_re_xb()
%log = RUN_RE_XB() Run UAV remote with Xbox control
%   
%   Outputs:
%   - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('uav.interfaces.Remote');
import('uav.cmd_src.Xbox');
import('uav.scripts.run');

% Function
clc
instrreset
uav = Remote();
cmd_src = Xbox();
run_gui = true;
save_log = true;
log = run(uav, cmd_src, run_gui, save_log);

end