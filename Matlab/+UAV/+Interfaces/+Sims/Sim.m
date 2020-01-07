classdef (Abstract) Sim < UAV.Interfaces.Interface
    %SIM Superclass for simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        phys;   % UAV physical model [UAV.Models.Phys]
        f_sim;  % Simulation frequency [Hz]
        t_sim;  % Simulation period [s]
    end
    
    methods (Access = public)
        function obj = Sim(phys, f_sim)
            %obj = SIM(phys, f_sim) Construct UAV simulator
            %   Inputs:
            %       phys = UAV physical model [UAV.Models.Phys]
            %       f_sim = Simulation frequency [Hz]
            obj = obj@UAV.Interfaces.Interface();
            obj.phys = phys;
            obj.f_sim = f_sim;
            obj.t_sim = 1 / f_sim;
        end
    end
    
    methods (Access = protected)
        function update_sim(obj, f_prop)
            %UPDATE_SIM(obj, f_prop) Run one simulation iteration
            %   Inputs:
            %       f = Prop force vector [N]
            
            % Angular dynamics
            ang_acc = obj.model.phys.M_bar_ang * f_prop;
            ang_vel = obj.state.ang_vel;
            norm_ang_vel = norm(ang_vel);
            if norm_ang_vel > 0
                theta = norm_ang_vel * obj.t_sim;
                ang_pos = obj.state.ang_pos * Quat(ang_vel, theta);
            end
            ang_vel = ang_vel + ang_acc * obj.t_sim;
            
            % Linear dynamics
            lin_acc = obj.phys.M_bar_lin * f_prop;
            lin_acc = [0; 0; lin_acc];
            lin_acc = ang_pos.rotate(lin_acc) - obj.phys.g_vec;
            
            % State machine
            z_hat = ang_pos.rotate([0; 0; 1]);
            if z_hat(3) > 0
                state = 'Enabled';
            else
                state = 'Failed';
            end
            
            % Set state
            obj.state = State(ang_pos, ang_vel, lin_acc, f_prop, state);
        end
    end
end

