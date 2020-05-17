classdef (Abstract) Sim < uav.interface.Interface
    %SIM Superclass for simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        I_mat;  % Inertia matrix [kg*m^2]
        A_ang;  % Angular accel matrix [(rad/s^2)/thr]
        A_lin;  % Linear accel matrix [(m/s^2)/thr]
    end
    
    methods (Access = public)
        function obj = Sim(model, params)
            %obj = SIM(model, params)
            %   Construct UAV simulator
            %   
            %   Inputs:
            %   - model = UAV model [uav.Model]
            %   - params = Flight params [uav.Params]
            
            % Superconstructor
            obj@uav.interface.Interface(model, params);
            
            % Acceleration matrices
            obj.I_mat = diag([model.inr_xx, model.inr_yy, model.inr_zz]);
            D_mat = [...
                [+model.rad_x, -model.rad_x, +model.rad_x, -model.rad_x]; ...
                [-model.rad_y, -model.rad_y, +model.rad_y, +model.rad_y]; ...
                [+model.rad_z, -model.rad_z, -model.rad_z, +model.rad_z]];
            obj.A_ang = model.f_max * inv(obj.I_mat) * D_mat;
            obj.A_lin = zeros(3, 4);
            obj.A_lin(3, :) = model.f_max / model.mass;
        end
    end
    
    methods (Access = protected)
        function state = update_sim(obj, thr_props, enum_cmd)
            %state = UPDATE_SIM(obj, thr_props, enum_cmd)
            %   Run one simulation iteration
            %   
            %   Inputs:
            %   - thr_props = Prop throttle vector [N]
            %   - enum_cmd = State machine enum cmd [uav.state.Enum]
            %   
            %   Outputs:
            %   - state = UAV state [uav.state.State]
            
            % Imports
            import('uav.state.State');
            import('uav.state.Enum');
            
            % State machine
            ang_pos = obj.state.ang_pos;
            z_hat = ang_pos.rotate([0; 0; 1]);
            if z_hat(3) < 0
                enum_cmd = Enum.Failed;
            end
            if enum_cmd ~= Enum.Enabled
                thr_props = zeros(4, 1);
            end
            
            % Angular dynamics
            ang_vel = obj.state.ang_vel;
            ang_acc = obj.A_ang * thr_props - ...
                obj.I_mat \ cross(ang_vel, obj.I_mat * ang_vel);
            norm_ang_vel = norm(ang_vel);
            if norm_ang_vel > 0
                theta = norm_ang_vel * obj.model.t_ctrl;
                ang_pos = pos_w(ang_pos * quat.Quat(ang_vel, theta));
            end
            ang_vel = ang_vel + ang_acc * obj.model.t_ctrl;
            
            % Linear dynamics
            lin_acc = obj.A_lin * thr_props;
            lin_acc = ang_pos.rotate(lin_acc) - obj.model.gravity_vec;
            
            % Update state
            obj.state = State(ang_pos, ang_vel, lin_acc, thr_props, enum_cmd);
            state = obj.state;
        end
    end
end