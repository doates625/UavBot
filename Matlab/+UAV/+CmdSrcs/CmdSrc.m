classdef (Abstract) CmdSrc < handle
    %CMDSRC Superclass for UAV command sources
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public, Abstract)
        [cmd, time] = get_cmd(obj)
        %[cmd, time] = GET_CMD(obj) Get commands and time
        %   Outputs:
        %       cmd = UAV command [UAV.State.Cmd]
        %       time = Time [s]
        
        stop = get_stop(obj)
        %stop = GET_STOP(obj) Get stop flag [logical]
    end
end