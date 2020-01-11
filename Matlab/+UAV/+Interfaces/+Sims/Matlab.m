classdef Matlab < UAV.Interfaces.Sims.Sim
    %MATLAB Matlab simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        acc_mag_min;    % Min linear acceleration magnitude [m/s^2]
        acc_mag_max;    % Max linear acceleration magnitude [m/s^2]
        acc_mag_max_sq; % Max linear accel magnitude squared [(m/s^2)^2]
        quat_x_pid;     % Quaternion x-axis ctrl [PID]
        quat_y_pid;     % Quaternion y-axis ctrl [PID]
        quat_z_pid;     % Quaternion z-axis ctrl [PID]
        quat_sat;       % Quaternion PID saturation flag [logical]
        acc_z_pid;      % Acceleration local-z ctrl [PID]
    end
    
    methods (Access = public)
        function obj = Matlab(model)
            %obj = MATLAB(model) Construct Matlab simulator with given model [UAV.Model]
            
            % Superconstructor
            if nargin < 1, model = UAV.Model(); end
            obj = obj@UAV.Interfaces.Sims.Sim(model);
            
            % Acceleration limits
            acc_max = 4 * obj.model.f_prop_max / obj.model.mass;
            obj.acc_mag_min = obj.model.f_rat_min * acc_max;
            obj.acc_mag_max = obj.model.f_rat_max * acc_max;
            obj.acc_mag_max_sq = obj.acc_mag_max^2;
            
            % Quaternion PID controllers
            obj.quat_x_pid = PID(...
                obj.model.qx_kp, obj.model.qx_ki, obj.model.qx_kd, ...
                -realmax(), +realmax(), obj.f_sim);
            obj.quat_y_pid = PID(...
                obj.model.qy_kp, obj.model.qy_ki, obj.model.qy_kd, ...
                -realmax(), +realmax(), obj.f_sim);
            obj.quat_z_pid = PID(...
                obj.model.qz_kp, obj.model.qz_ki, obj.model.qz_kd, ...
                -realmax(), +realmax(), obj.f_sim);
            obj.quat_sat = false;
            
            % Acceleration PID controller
            f_lin_min = 4 * obj.model.f_prop_max * obj.model.f_rat_min;
            f_lin_max = 4 * obj.model.f_prop_max * obj.model.f_rat_max;
            obj.acc_z_pid = PID(...
                obj.model.az_kp, obj.model.az_ki, obj.model.az_kd, ...
                f_lin_min, f_lin_max, obj.f_sim);
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   Inputs:
            %       cmd = UAV command [UAV.State.Cmd]
            %   Outputs:
            %       state = UAV state [UAV.State.State]
            
            % Simulate controller
            if cmd.enum == UAV.State.Enum.Enabled
                [quat_cmd, acc_z_cmd] = obj.lap(cmd.lin_acc, cmd.ang_z);
                f_ang = obj.qoc(quat_cmd);
                f_lin = obj.lac(acc_z_cmd);
                f_prop = obj.frc(f_ang, f_lin);
            else
                f_prop = zeros(4, 1);
            end
            
            % Simulate dynamics
            state = obj.update_sim(f_prop, cmd.enum);
        end
    end
    
    methods (Access = protected)
        function [quat_cmd, acc_z_cmd] = lap(obj, lin_acc_cmd, ang_z_cmd)
            %[quat_cmd, acc_z_cmd] = LAP(obj, lin_acc_cmd, ang_z_cmd)
            %   Linear Acceleration Planner
            %   Inputs:
            %       lin_acc_cmd = Global linear acceleration cmd [m/s^2]
            %       ang_z_cmd = Heading cmd [rad]
            %   Outputs:
            %       quat_cmd = Orientation cmd [Quat]
            %       acc_z_cmd = Local z-axis acceleration cmd [m/s^2]
            
            % Adjust for gravity
            acc_cmd_adj = lin_acc_cmd + obj.model.gravity_vec;
            
            % Acceleration limiting
            acc_cmd_adj(3) = clamp(acc_cmd_adj(3), obj.acc_mag_min, obj.acc_mag_max);
            norm_xy = norm(acc_cmd_adj(1:2));
            norm_xy_max = sqrt(obj.acc_mag_max_sq - acc_cmd_adj(3)^2);
            p = norm_xy_max / norm_xy;
            if p < 1
                acc_cmd_adj(1:2) = p * acc_cmd_adj(1:2);
            end
            
            % Orientation
            quat_z = Quat([0; 0; 1], ang_z_cmd);
            norm_acc = norm(acc_cmd_adj);
            if norm_acc > 0
                acc_hat = acc_cmd_adj / norm_acc;
                c_z = cos(ang_z_cmd);
                s_z = sin(ang_z_cmd);
                ang_x = asin(s_z*acc_hat(1) - c_z*acc_hat(2));
                ang_y = asin((c_z*acc_hat(1) + s_z*acc_hat(2))/cos(ang_x));
                quat_y = Quat([0; 1; 0], ang_y);
                quat_x = Quat([1; 0; 0], ang_x);
                quat_cmd = quat_z * quat_y * quat_x;
            else
                quat_cmd = quat_z;
            end
            
            % Acceleration z-axis cmd
            acc_cmd_lim = acc_cmd_adj - obj.model.gravity_vec;
            acc_cmd_loc = obj.state.ang_pos.inv().rotate(acc_cmd_lim);
            acc_z_cmd = acc_cmd_loc(3);
        end
        
        function f_ang = qoc(obj, quat_cmd)
            %f_ang = QOC(obj, quat_cmd)
            %   Quaternion Orientation Controller
            %   Inputs:
            %       quat_cmd = Orientation cmd [Quat]
            %   Outputs:
            %       f_ang = Angular prop force vector [N]
            
            % Compute error quaternion
            quat_err = quat_cmd \ obj.state.ang_pos;
            if quat_err.w < 0
                quat_err = -quat_err;
            end
            
            % Compute net torque cmd
            tau_cmd = zeros(3, 1);
            tau_cmd(1) = obj.quat_x_pid.update(-quat_err.x, 0, obj.quat_sat);
            tau_cmd(2) = obj.quat_y_pid.update(-quat_err.y, 0, obj.quat_sat);
            tau_cmd(3) = obj.quat_z_pid.update(-quat_err.z, 0, obj.quat_sat);
            
            % Compute force cmd
            f_ang = obj.model.D_inv_ang * tau_cmd;
        end
        
        function f_lin = lac(obj, acc_z_cmd)
            %f_lin = LAC(obj, acc_mag_cmd)
            %   Local acceleration controller
            %   Inputs:
            %       acc_z_cmd = Local z-axis acceleration cmd [m/s^2]
            %   Outputs:
            %       f_lin = Linear prop force vector [N]
            acc_glo = obj.state.lin_acc;
            acc_loc = obj.state.ang_pos.inv().rotate(acc_glo);
            f_lin = obj.model.D_inv_lin * obj.acc_z_pid.update(acc_z_cmd - acc_loc(3));
        end
        
        function f_prop = frc(obj, f_ang, f_lin)
            %f_prop = FRC(obj, f_ang, f_lin)
            %   Force Regulator Controller
            %   Inputs:
            %       f_ang = Angular prop forces [N]
            %       f_lin = Linear prop forces [N]
            %   Outputs:
            %       f_prop = Combined prop forces [N]
            
            % Angular force limit
            p_min = 1;
            obj.quat_sat = false;
            for i = 1:4
                if f_ang(i) > 0
                    p = (obj.model.f_prop_max - f_lin(i)) / f_ang(i);
                elseif f_ang(i) < 0
                    p = (obj.model.f_prop_min - f_lin(i)) / f_ang(i);
                else
                    p = 1;
                end
                if 0 < p && p < p_min
                    p_min = p;
                    obj.quat_sat = true;
                end
            end
            
            % Combine forces
            f_prop = p_min * f_ang + f_lin;
        end
    end
end