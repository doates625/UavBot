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
import('uav.cmd_src.Xbox');
import('uav.interface.sim.Matlab');
import('uav.Gui');
import('uav.logging.Log');

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

% Close GUI
if run_gui, delete(gui); end

% Log processing
log.trim();
if save_log, log.save(); end
log.plot();

end