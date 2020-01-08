classdef Matlab < UAV.Interfaces.Sims.Sim
    %MATLAB Matlab simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        ctrl_model;     % UAV control model [UAV.Models.Ctrl]
        acc_mag_min;    % Min linear acceleration magnitude [m/s^2]
        acc_mag_max;    % Max linear acceleration magnitude [m/s^2]
        acc_mag_max_sq; % Max linear accel magnitude squared [(m/s^2)^2]
        quat_x_pid;     % Quaternion x-axis PID controller [PID]
        quat_y_pid;     % Quaternion y-axis PID controller [PID]
        quat_z_pid;     % Quaternion z-axis PID controller [PID]
        quat_sat;       % Quaternion PID saturation flag [logical]
    end
    
    methods (Access = public)
        function obj = Matlab(phys_model, ctrl_model, f_sim)
            %obj = MATLAB(phys_model, ctrl_model, f_sim)
            %   Construct matlab simulator
            %   Inputs:
            %       phys_model = UAV physical model [UAV.Models.Phys]
            %       ctrl_model = UAV control model [UAV.Models.Ctrl]
            %       f_sim = Simulation frequency [Hz]
            
            % Default args
            import('UAV.default_arg')
            if nargin < 3, f_sim = default_arg('f_sim'); end
            if nargin < 2, ctrl_model = default_arg('ctrl_model'); end
            if nargin < 1, phys_model = default_arg('phys_model'); end
            
            % Superconstructor
            obj = obj@UAV.Interfaces.Sims.Sim(phys_model, f_sim);
            obj.ctrl_model = ctrl_model;
            
            % Acceleration limits
            acc_max = 4 * obj.phys_model.f_max / obj.phys_model.mass;
            obj.acc_mag_min = obj.ctrl_model.fr_min * acc_max;
            obj.acc_mag_max = obj.ctrl_model.fr_max * acc_max;
            obj.acc_mag_max_sq = obj.acc_mag_max^2;
            
            % Quaternion PID controllers
            obj.quat_x_pid = obj.quat_pid(obj.phys_model.I_xx, obj.ctrl_model.s_qx);
            obj.quat_y_pid = obj.quat_pid(obj.phys_model.I_yy, obj.ctrl_model.s_qy);
            obj.quat_z_pid = obj.quat_pid(obj.phys_model.I_zz, obj.ctrl_model.s_qz);
            
            % Quaternion saturation flag
            obj.quat_sat = false;
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   Inputs:
            %       cmd = UAV command [UAV.Cmd]
            %   Outputs:
            %       state = UAV state [UAV.State]
            
            % Simulate controller
            [q_cmd, acc_mag] = obj.lap(cmd.acc, cmd.tz);
            f_ang = obj.qoc(q_cmd);
            f_lin = obj.phys_model.M_lin * acc_mag;
            f_prop = obj.frc(f_ang, f_lin);
            
            % Simulate dynamics
            state = obj.update_sim(f_prop, cmd.state);
        end
    end
    
    methods (Access = protected)
        function [q_cmd, acc_mag] = lap(obj, acc_cmd, tz_cmd)
            %[q_cmd, acc_mag] = LAP(obj, acc_cmd, tz_cmd)
            %   Linear Acceleration Planner
            %   Inputs:
            %       acc_cmd = Global acceleration cmd [m/s^2]
            %       tz_cmd = Heading cmd [rad]
            %   Outputs:
            %       q_cmd = Orientation cmd [Quat]
            %       acc_mag = Accel magnitude [m/s^2]
            
            % Adjust for gravity
            acc_cmd = acc_cmd + obj.phys_model.g_vec;
            
            % Acceleration limiting
            acc_cmd(3) = clamp(acc_cmd(3), obj.acc_mag_min, obj.acc_mag_max);
            norm_xy = norm(acc_cmd(1:2));
            norm_xy_max = sqrt(obj.acc_mag_max_sq - acc_cmd(3)^2);
            p = norm_xy_max / norm_xy;
            if p < 1
                acc_cmd(1:2) = p * acc_cmd(1:2);
            end
            
            % Orientation
            qz = Quat([0; 0; 1], tz_cmd);
            norm_acc = norm(acc_cmd);
            if norm_acc > 0
                acc_hat = acc_cmd / norm_acc;
                cz = cos(tz_cmd);
                sz = sin(tz_cmd);
                tx = asin(sz*acc_hat(1) - cz*acc_hat(2));
                ty = asin((cz*acc_hat(1) + sz*acc_hat(2))/cos(tx));
                qy = Quat([0; 1; 0], ty);
                qx = Quat([1; 0; 0], tx);
                q_cmd = qz * qy * qx;
            else
                q_cmd = qz;
            end
            
            % Acceleration magnitude
            z_hat = [0; 0; 1];
            n_hat = obj.state.ang_pos.rotate(z_hat);
            acc_mag = acc_cmd(3) / n_hat(3);
            acc_mag = clamp(acc_mag, obj.acc_mag_min, obj.acc_mag_max);
        end
        
        function f_ang = qoc(obj, q_cmd)
            %f_ang = QOC(obj, q_cmd)
            %   Quaternion Orientation Controller
            %   Inputs:
            %       q_cmd = Orientation cmd [Quat]
            %   Outputs:
            %       f_ang = Angular prop forces [N]
            
            % Compute error quaternion
            q_err = q_cmd \ obj.state.ang_pos;
            if q_err.w < 0
                q_err = -q_err;
            end
            
            % Compute net torque cmd
            tau = zeros(3, 1);
            tau(1) = obj.quat_x_pid.update(-q_err.x, 0, obj.quat_sat);
            tau(2) = obj.quat_y_pid.update(-q_err.y, 0, obj.quat_sat);
            tau(3) = obj.quat_z_pid.update(-q_err.z, 0, obj.quat_sat);
            
            % Compute force cmd
            f_ang = obj.phys_model.D_bar_ang * tau;
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
                    p = (obj.phys_model.f_max - f_lin(i)) / f_ang(i);
                elseif f_ang(i) < 0
                    p = (obj.phys_model.f_min - f_lin(i)) / f_ang(i);
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
        
        function pid = quat_pid(obj, I, s)
            %pid = QUAT_PID(obj, I, s) Make quaternion PID controller
            %   Inputs:
            %       I = Axis inertia [kg*m^2]
            %       s = Feedback triple pole [s^-1]
            %   Outputs:
            %       pid = PID controller [PID]
            kp = 6*I*s^2;
            ki = -2*I*s^3;
            kd = -6*I*s;
            u_min = -realmax();
            u_max = +realmax();
            pid = PID(kp, ki, kd, u_min, u_max, obj.f_sim);
        end
    end
end

