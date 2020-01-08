function run(uav, cmd_src)
%RUN(uav, cmd_src, acc_max, wz_max) Run UAV interface
%   Inputs:
%       uav = UAV interface [UAV.Interfaces.Interface]
%       cmd_src = Command source [UAV.CmdSrcs.CmdSrc]
%   Author: Dan Oates (WPI Class of 2020)

% Default args
if nargin < 2, cmd_src = UAV.CmdSrcs.Xbox(); end
if nargin < 1, uav = UAV.Interfaces.Sims.Matlab(); end

% Create GUI
fig = figure(1);
gui = UAV.Gui(fig);

% Loop timer
timer = Timer();

% Command loop
while true
    
    % Loop routines
    t = timer.toc();
    cmd = cmd_src.get_cmd(t);
    state = uav.update(cmd);
    gui.update(state, cmd, t);
    drawnow
    
    % Exit condition
    if cmd_src.get_stop()
        break
    end
end

end