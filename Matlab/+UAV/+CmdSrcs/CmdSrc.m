classdef (Abstract) CmdSrc < handle
    %CMDSRC Superclass for UAV command sources
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public, Abstract)
        [cmd, t] = get_cmd(obj)
        %[cmd, t] = GET_CMD(obj) Get commands and time
        %   Outputs:
        %       cmd = UAV command [UAV.Cmd]
        %       t = Time [s]
        
        stop = get_stop(obj)
        %stop = GET_STOP(obj) Get stop flag [logical]
    end
end