classdef (Abstract) CmdSrc < handle
    %CMDSRC Superclass for UAV command sources
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public, Abstract)
        cmd = get_cmd(obj, t)
        %cmd = GET_CMD(obj, t) Get commands
        %   Inputs:
        %       t = Time [s]
        %   Outputs:
        %       cmd = UAV command [UAV.Cmd]
        
        stop = get_stop(obj)
        %stop = GET_STOP(obj) Get stop flag [logical]
    end
end