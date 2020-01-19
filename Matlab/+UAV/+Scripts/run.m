function log = run(uav, cmd_src, run_gui, save_log)
%log = RUN(uav, cmd_src, run_gui, save_log) Run UAV interface
%   
%   Inputs:
%   - uav = UAV interface [UAV.Interfaces.Interface]
%   - cmd_src = Command source [UAV.CmdSrcs.CmdSrc]
%   - run_gui = Flag to run GUI [logical]
%   - save_log = Flag to save log file [logical]
%   
%   Outputs:
%   - log = Flight log object [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('UAV.CmdSrcs.Xbox');
import('UAV.Interfaces.Sims.Matlab');
import('UAV.Gui');
import('UAV.Logging.Log');

% Default args
if nargin < 4, save_log = false; end
if nargin < 3, run_gui = true; end
if nargin < 2, cmd_src = Xbox(); end
if nargin < 1, uav = Matlab(); end

% Initializations
if run_gui, gui = Gui(uav); end
log = Log(uav);

% Command loop
while true
    
    % Loop routines
    [cmd, time] = cmd_src.get_cmd();
    uav.update(cmd);
    if run_gui, gui.update(time); end
    log.update(time);
    drawnow
    
    % Exit condition
    if cmd_src.get_stop()
        break
    end
end

% Log processing
log.trim();
if save_log, log.save(); end
log.plot();

end