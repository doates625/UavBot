classdef (Abstract) Sim < UAV.Interfaces.Interface
    %SIM Superclass for simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = public, Constant)
        f_sim = 50.0;   % Simulation frequency [Hz]
        t_sim = 0.02;   % Simulation period [s]
    end
    
    methods (Access = public)
        function obj = Sim(phys_model)
            %obj = SIM(phys_model) Construct UAV simulator
            %   Inputs:
            %       phys_model = UAV physical model [UAV.Models.Phys]
            obj = obj@UAV.Interfaces.Interface(phys_model);
        end
    end
    
    methods (Access = protected)
        function state = update_sim(obj, f_props, enum_cmd)
            %state = UPDATE_SIM(obj, f_props, enum_cmd) Run one simulation iteration
            %   Inputs:
            %       f_props = Prop force vector [N]
            %       enum_cmd = State machine enum cmd [UAV.State.Enum]
            %   Outputs:
            %       state = UAV state [UAV.State.State]
            
            % State machine
            import('UAV.State.Enum');
            ang_pos = obj.state.ang_pos;
            z_hat = ang_pos.rotate([0; 0; 1]);
            if z_hat(3) > 0
                enum = enum_cmd;
            else
                enum = Enum.Failed;
            end
            if enum ~= Enum.Enabled
                f_props = zeros(4, 1);
            end
            
            % Angular dynamics
            ang_acc = obj.phys_model.M_bar_ang * f_props;
            ang_vel = obj.state.ang_vel;
            norm_ang_vel = norm(ang_vel);
            if norm_ang_vel > 0
                theta = norm_ang_vel * obj.t_sim;
                ang_pos = ang_pos * Quat(ang_vel, theta);
            end
            ang_vel = ang_vel + ang_acc * obj.t_sim;
            
            % Linear dynamics
            lin_acc = obj.phys_model.M_bar_lin * f_props;
            lin_acc = [0; 0; lin_acc];
            lin_acc = ang_pos.rotate(lin_acc) - obj.phys_model.g_vec;
            
            % Set state
            obj.state = UAV.State.State(ang_pos, ang_vel, lin_acc, f_props, enum);
            state = obj.state;
        end
    end
end