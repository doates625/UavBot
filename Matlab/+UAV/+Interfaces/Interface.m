classdef (Abstract) Interface < handle
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
        state = update(obj, cmd);
        %state = UPDATE(obj, cmd)
        %   Send commands and get new state
        %   Inputs:
        %       cmd = UAV command [UAV.Cmd]
        %   Outputs:
        %       state = UAV state [UAV.State]
    end
end