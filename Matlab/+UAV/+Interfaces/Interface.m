classdef (Abstract) Interface
    %INTERFACE Superclass for UAV interfaces (real and simulated)
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
    	state;  % UAV state [UAV.State]
    end
    
    methods (Access = public)
        function obj = Interface()
            %obj = INTERFACE() Construct UAV interface
            obj.state = UAV.State();
        end
    end
    
    methods (Access = public, Abstract)
        state = update(obj, lin_acc_cmd, tz_cmd);
        %state = UPDATE(obj, acc_cmd, tz_cmd)
        %   Send commands and get new state
        %   Inputs:
        %       lin_acc_cmd = Global acceleration cmd [m/s^2]
        %       ang_z_cmd = Heading cmd [rad]
        %   Outputs:
        %       state = UAV state [UAV.State]
    end
end