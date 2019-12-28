classdef (Abstract) UAVInterface < handle
    %UAVINTERFACE Interface for actual and simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model   % UAV model [UAVModel]
    end
    
    methods (Abstract)
        [q, w, f, acc, tz, q_cmd] = update(obj, acc_cmd, tz_cmd)
    end
    
    methods
        function obj = UAVInterface(model)
            %obj = UAVInterface(model) Constructor
            %   model = UAV model [UAVModel]
            obj.model = model;
        end
    end
end