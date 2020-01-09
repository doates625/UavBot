classdef (Abstract) Interface < handle
    %INTERFACE Superclass for UAV interfaces (real and simulated)
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        phys_model; % UAV physical model [UAV.Models.Phys]
    	state;      % UAV state [UAV.State.State]
    end
    
    methods (Access = public)
        function obj = Interface(phys_model)
            %obj = INTERFACE() Construct UAV interface
            %   Inputs:
            %       phys_model = UAV physical model [UAV.Models.Phys]
            obj.phys_model = phys_model;
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