classdef (Abstract) UavInterface < handle
    %UAVINTERFACE Class for actual and simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model;  % UAV model [UAVModel]
    end
    
    methods (Abstract)
        [q, w, acc, tz, f, stat] = update(obj, acc_cmd, tz_cmd);  
    end
    
    methods (Access = public)
        function obj = UavInterface(model)
            %obj = UAVINTERFACE(model) Constructor
            %   model = UAV model [UAVModel]
            obj.model = model;
        end
    end
end