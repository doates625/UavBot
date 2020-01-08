function log = run(uav, cmd_src, run_gui)
%log = RUN(uav, cmd_src) Run UAV interface
%   Inputs:
%       uav = UAV interface [UAV.Interfaces.Interface]
%       cmd_src = Command source [UAV.CmdSrcs.CmdSrc]
%       run_gui = Flag to run GUI [logical]
%   Outputs:
%       log = Flight log object [UAV.Log]
%   Author: Dan Oates (WPI Class of 2020)
close, clc

% Default args
if nargin < 3, run_gui = true; end
if nargin < 2, cmd_src = UAV.CmdSrcs.Xbox(); end
if nargin < 1, uav = UAV.Interfaces.Sims.Matlab(); end

% Initializations
if run_gui, gui = UAV.Gui(); end
log = UAV.Log();

% Command loop
first_loop = true;
while true
    
    % Loop routines
    [cmd, t] = cmd_src.get_cmd();
    state = uav.update(cmd);
    if run_gui, gui.update(state, cmd, t); end
    log.update(state, cmd, t);
    drawnow
    
    % Xbox init override
    if first_loop && isa(cmd_src, 'UAV.CmdSrcs.Xbox')
        cmd_src.set_tz(state.get_tz());
        first_loop = false;
    end
    
    % Exit condition
    if cmd_src.get_stop()
        break
    end
end

% Plot log
log.plot();

end