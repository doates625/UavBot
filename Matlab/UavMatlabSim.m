classdef UavMatlabSim < UavSim
    %UAVMATLABSIM Matlab simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        a_min;      % Min accel cmd [m/s^2]
        a_max;      % Max accel cmd [m/s^2]
        q_x_pid;    % Quat-x PID [PID]
        q_y_pid;    % Quat-y PID [PID]
        q_z_pid;    % Quat-z PID [PID]
        q_sat;      % Quat PID saturation [logical]
    end
    
    methods (Access = public)
        function obj = UavMatlabSim(model, f_sim, q_pole, th_min, th_max)
            %obj = UAVMATLABSIM(model, f_sim, q_pole, th_min, th_max)
            %   Construct UAV Matlab simulator
            %   Inputs:
            %       model = UAV model [UAVModel]
            %       f_sim = Sim frequency [Hz]
            %       q_pole = Quat ctrl pole [s^-1]
            %       th_min = Min thrust ratio [0-1]
            %       th_max = Max thrust ratio [0-1] 
            
            % Superconstructor
            obj = obj@UavSim(model, f_sim);
            
            % Acceleration limits
            acc_max = 4 * obj.model.f_max / obj.model.mass;
            obj.a_min = th_min * acc_max;
            obj.a_max = th_max * acc_max;
            
            % Quat-x PID
            obj.q_x_pid = obj.make_quat_pid(obj.model.I_xx, q_pole);
            obj.q_y_pid = obj.make_quat_pid(obj.model.I_yy, q_pole);
            obj.q_z_pid = obj.make_quat_pid(obj.model.I_zz, q_pole);
            obj.q_sat = false;
        end
        
        function [q, w, acc, tz, f, stat] = update(obj, acc_cmd, tz_cmd)
            %[q, w, acc, tz, f, stat] = UPDATE(obj, acc_cmd, tz_cmd)
            %   Run simulation iteration and get states
            %   Inputs:
            %       acc_cmd = Global accel cmd [m/s^2]
            %       tz_cmd = Heading cmd [rad]
            %   Outputs:
            %       q = Orientation [Quat]
            %       w = Local angular velocity [rad/s]
            %       acc = Global accel [m/s^2]
            %       tz = Heading [rad]
            %       f = Propeller forces [N]
            %       stat = Status [0 = OK, 1 = failed]
            
            % Simulate controller
            [q_cmd, acc_mag] = obj.lap(acc_cmd, tz_cmd);
            f_alp = obj.qoc(q_cmd);
            f = obj.frc(f_alp, acc_mag);
            
            % Simulate dynamics
            [q, w, acc] = obj.update_sim(f);
            [tz, stat] = obj.proc_quat();
        end
    end
    
    methods (Access = protected)
        function [q_cmd, acc_mag] = lap(obj, acc_cmd, tz_cmd)
            %[q_cmd, acc_mag] = LAP(obj, acc_cmd, tz_cmd)
            %   Linear Acceleration Planner
            %   Inputs:
            %       acc_cmd = Global accel cmd [m/s^2]
            %       tz_cmd = Heading cmd [rad]
            %   Outputs:
            %       q_cmd = Orientation cmd [Quat]
            %       acc_mag = Accel magnitude [m/s^2]
            
            % Adjust for gravity
            acc_cmd = acc_cmd + obj.model.g_vec;
            
            % Acceleration limiting
            acc_cmd(3) = clamp(acc_cmd(3), obj.a_min, obj.a_max);
            norm_xy = norm(acc_cmd(1:2));
            norm_xy_max = sqrt(obj.a_max^2 - acc_cmd(3)^2);
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
            
            % Acceleration
            z_hat = [0; 0; 1];
            n_hat = obj.q.rotate(z_hat);
            acc_mag = acc_cmd(3) / n_hat(3);
            acc_mag = clamp(acc_mag, obj.a_min, obj.a_max);
        end
        
        function f_alp = qoc(obj, q_cmd)
            %f_alp = QOC(obj, q_cmd)
            %   Quaternion Orientation Controller
            %   Inputs:
            %       q_cmd = Orientation cmd [Quat]
            %   Outputs:
            %       f_alp = Differential prop forces [N]
            
            % Compute error vector
            q_err = q_cmd \ obj.q;
            if q_err.w < 0
                q_err = -q_err;
            end
            
            % Compute net torque cmd
            tau = zeros(3, 1);
            tau(1) = obj.q_x_pid.update(-q_err.x, 0, obj.q_sat);
            tau(2) = obj.q_y_pid.update(-q_err.y, 0, obj.q_sat);
            tau(3) = obj.q_z_pid.update(-q_err.z, 0, obj.q_sat);
            
            % Compute force cmd
            f_alp = obj.model.D_bar * tau;
        end
        
        function f = frc(obj, f_alp, acc_mag)
            %f = FRC(obj, f_alp, acc_mag)
            %   Force Regulator Controller
            %   Inputs:
            %       f_alp = Differential prop forces [N]
            %       acc_mag = Accel magnitude [m/s^2]
            %   Outputs:
            %       f = Prop force vector [N] 
            
            % Apply angular limit
            f_acc = obj.model.M_acc * acc_mag;
            p_min = 1;
            for i = 1:4
                if f_alp(i) > 0
                    p = (obj.model.f_max - f_acc(i)) / f_alp(i);
                elseif f_alp(i) < 0
                    p = (obj.model.f_min - f_acc(i)) / f_alp(i);
                else
                    p = 1;
                end
                if 0 < p && p < p_min
                    p_min = p;
                end
            end
            f = p_min * f_alp + f_acc;
            
            % Saturation flag
            if p_min < 1
                obj.q_sat = true;
            end
            
            % Apply clamp limit (TODO remove and see if OK?)
            for i = 1:4
                f(i) = clamp(f(i), obj.model.f_min, obj.model.f_max);
            end
        end
        
        function pid = make_quat_pid(obj, I, s)
            %pid = MAKE_QUAT_PID(I, q_pole) Make quat PID controller
            %   Inputs:
            %       I = Axis inertia [kg*m^2]
            %       s = Triple pole [s^-1]
            %   Outputs:
            %       pid = PID controller [PID]
            kp = 3*I*s^2;
            ki = -I*s^3;
            kd = -3*I*s;
            u_min = -realmax();
            u_max = +realmax();
            pid = PID(kp, ki, kd, u_min, u_max, obj.f_sim);
        end
    end
end