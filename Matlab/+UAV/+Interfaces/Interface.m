classdef (Abstract) Interface < handle
    %INTERFACE Superclass for UAV interfaces (real and simulated)
    %   Author: Dan Oates (WPI Class of 2020)

    properties (SetAccess = protected)
        model;  % UAV model [UAV.Model]
        params; % UAV flight parameters [UAV.Params]
    	state;  % UAV state [UAV.State.State]
    end
    
    methods (Access = public)
        function obj = Interface(model, params)
            %obj = INTERFACE(model, param_file) Construct UAV interface
            %   Inputs:
            %       model = UAVmodel [UAV.Model]
            %       params = Flight params file [char]
            % 
            obj.model = model;
            obj.params = params;
            obj.state = UAV.State.State();
        end
    end
    
    methods (Access = public)
        
    end
    
    methods (Access = public, Abstract)
        state = update(obj, cmd);
        %state = UPDATE(obj, cmd)
        %   Send commands and get new state
        %   Inputs:
        %       cmd = UAV command [UAV.State.Cmd]
        %   Outputs:
        %       state = UAV state [UAV.State.State]
        
        set_params(obj, params)
        %SET_PARAMS(obj, params)
        %   Set flight parameters
        %   Inputs:
        %       params = Flight params [UAV.Params]
    end
end