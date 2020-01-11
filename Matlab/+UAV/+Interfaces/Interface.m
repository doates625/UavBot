classdef (Abstract) Interface < handle
    %INTERFACE Superclass for UAV interfaces (real and simulated)
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model;  % UAV model [UAV.Model]
    	state;  % UAV state [UAV.State.State]
    end
    
    methods (Access = public)
        function obj = Interface(model)
            %obj = INTERFACE(model) Construct UAV interface with given model [UAV.Model]
            obj.model = model;
            obj.state = UAV.State.State();
        end
    end
    
    methods (Access = public, Abstract)
        state = update(obj, cmd);
        %state = UPDATE(obj, cmd)
        %   Send commands and get new state
        %   Inputs:
        %       cmd = UAV command [UAV.State.Cmd]
        %   Outputs:
        %       state = UAV state [UAV.State.State]
    end
end