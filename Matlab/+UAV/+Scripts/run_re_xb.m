function log = run_re_xb()
%log = RUN_RE_XB() Run UAV remote with Xbox control
%   Outputs:
%       log = Flight log object [UAV.Log]
%   Author: Dan Oates (WPI Class of 2020)

clc
instrreset
uav = UAV.Interfaces.Remote();
cmd_src = UAV.CmdSrcs.Xbox();
run_gui = true;
log = UAV.Scripts.run(uav, cmd_src, run_gui);

end