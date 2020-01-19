classdef (Abstract) Interface < handle
    %INTERFACE Superclass for UAV interfaces (real and simulated)
    %   Author: Dan Oates (WPI Class of 2020)

    properties (SetAccess = protected)
        model;  % UAV model [uav.Model]
        params; % UAV flight parameters [uav.Params]
    	state;  % UAV state [uav.state.State]
        cmd;    % UAV command [uav.state.Cmd]
    end
    
    methods (Access = public)
        function obj = Interface(model, params)
            %obj = INTERFACE(model, param_file)
            %   Construct UAV interface
            %   
            %   Inputs:
            %   - model = UAVmodel [uav.Model]
            %   - params = Flight params file [char]
            import('uav.state.State');
            obj.model = model;
            obj.params = params;
            obj.state = State();
        end
    end

    methods (Access = public, Abstract)
        state = update(obj, cmd);
        %state = UPDATE(obj, cmd)
        %   Send commands and get new state
        %   
        %   Inputs:
        %   - cmd = UAV command [uav.state.Cmd]
        %   
        %   Outputs:
        %   - state = UAV state [uav.state.State]
        
        set_params(obj, params)
        %SET_PARAMS(obj, params)
        %   Set flight parameters
        %   
        %   Inputs:
        %   - params = Flight params [uav.Params]
    end
end