classdef (Abstract) UavSim < UavInterface
    %UAVSIM Class for simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        f_sim;  % Sim frequency [Hz]
        t_sim;  % Sim period [s]
    end
    
    methods
        function obj = UavSim(model, f_sim)
            %obj = UAVSIM(model, f_sim) Constructor
            %   model = UAV model [UavModel]
            %   f_sim = Sim frequency [Hz]
            obj = obj@UavInterface(model);
            obj.f_sim = f_sim;
            obj.t_sim = 1 / f_sim;
        end
    end
end