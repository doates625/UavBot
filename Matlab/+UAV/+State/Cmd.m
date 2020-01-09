classdef Cmd < handle
    %CMD Class for UAV commands
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        lin_acc;    % Global linear acceleration cmd [m/s^2]
        ang_z;      % Heading cmd [rad]
        enum;       % State machine enum cmd [UAV.State.Enum]
    end
    
    methods
        function obj = Cmd(lin_acc, ang_z, enum)
            %obj = CMD(lin_acc, ang_z, enum) Construct UAV command
            %   Inputs:
            %       lin_acc = Global linear acceleration cmd [m/s^2]
            %       ang_z = Heading cmd [rad]
            %       end = State machine enum cmd [UAV.State.Enum]
            obj.lin_acc = lin_acc;
            obj.ang_z = ang_z;
            obj.enum = enum;
        end
    end
end