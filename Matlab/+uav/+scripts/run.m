function log = run(uav, cmd_src, run_gui, gen_log)
%log = RUN(uav, cmd_src, run_gui, gen_log) Run UAV interface
%   
%   Inputs:
%   - uav = UAV interface [UAV.Interfaces.Interface]
%   - cmd_src = Command source [UAV.CmdSrcs.CmdSrc]
%   - run_gui = Flag to run GUI [logical]
%   - gen_log = Flag to generate log file [logical]
%   
%   Outputs:
%   - log = Flight log [UAV.Log]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('uav.cmd_src.Xbox');
import('uav.interface.sim.Matlab');
import('uav.Gui');
import('uav.logging.Log');

% Initializations
if run_gui, gui = Gui(uav); end
if gen_log, log = Log(uav); end

% Command loop
while true
    
    % Loop routines
    [cmd, time] = cmd_src.get_cmd();
    uav.update(cmd);
    if run_gui, gui.update(time); end
    if gen_log, log.update(time); end
    % drawnow
    
    % Exit condition
    if cmd_src.get_stop()
        break
    end
end

% Close GUI
if run_gui, delete(gui); end

% Log post-processing
if gen_log
    log.trim();
    log.save();
    log.plot();
else
    log = [];
end

end