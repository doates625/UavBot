classdef (Abstract) UavSim < UavInterface
    %UAVSIM Class for simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model;  % UAV model [UavModel]
        f_sim;  % Sim frequency [Hz]
        t_sim;  % Sim period [s]
    end
    
    methods (Access = public)
        function obj = UavSim(model, f_sim)
            %obj = UAVSIM(model, f_sim) Constructor
            %   model = UAV model [UavModel]
            %   f_sim = Sim frequency [Hz]
            obj = obj@UavInterface();
            obj.model = model;
            obj.f_sim = f_sim;
            obj.t_sim = 1 / f_sim;
        end
    end
    
    methods (Access = protected)  
        function [q, w, acc] = update_sim(obj, f)
            %[q, w, acc] = UPDATE_SIM(obj, f)
            %   Run one simulation iteration
            %   Inputs:
            %       f = Propeller forces [N]
            %   Outputs:
            %       q = Orientation [Quat]
            %       w = Local angular vel [rad/s]
            %       acc = Global accel [m/s^2]
            
            % DEBUG
            if ~isequal(size(f), [4, 1])
                disp('OOPS')
            end
            
            % Generalized accel vector
            a_gen = obj.model.M_mat \ f;
            
            % Global acceleration
            acc = [0; 0; a_gen(4)];
            acc = obj.q.rotate(acc) - obj.model.g_vec;
            
            % Simulate quat dynamics
            alp = a_gen(1:3);
            if norm(obj.w) > 0
                q_theta = norm(obj.w) * obj.t_sim;
                obj.q = unit(obj.q * Quat(obj.w, q_theta));
            end
            obj.w = obj.w + alp * obj.t_sim;
            q = obj.q;
            w = obj.w;
        end
    end
end