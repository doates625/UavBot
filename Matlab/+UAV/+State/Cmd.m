classdef Cmd < handle
    %CMD Class for UAV commands
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        ang_pos;    % Orientation cmd [Quat]
        thr_lin;    % Linear throttle cmd [0, 1]
        enum;       % State machine enum cmd [UAV.State.Enum]
    end
    
    methods
        function obj = Cmd(ang_pos, thr_lin, enum)
            %obj = CMD(ang_pos, thr_lin, enum) Construct UAV command
            %   Inputs:
            %       ang_pos = Orientation cmd [Quat]
            %       thr_lin = Linear throttle cmd [0, 1]
            %       end = State machine enum cmd [UAV.State.Enum]
            %   Arg ang_pos is formatted to be positive-w
            obj.ang_pos = pos_w(ang_pos);
            obj.thr_lin = thr_lin;
            obj.enum = enum;
        end
    end
end