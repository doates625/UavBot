classdef UAVSim < UAVInterface
    %UAVSIMULATOR Simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        f_sim   % Sim frequency [Hz]
        t_sim   % Sim period [s]
    end
    
    properties (GetAccess = protected)
        q;      % Orientation [Quat]
        w;      % Angular velocity [rad/s]
        k_q;    % Quaternion gain [rad/s^2]
        k_w;    % Velocity gain [s^-1]
        a_min;  % Min accel cmd [m/s^2]
        a_max;  % Max accel cmd [m/s^2]
    end
    
    methods (Access = public)
        function obj = UAVSim(model, f_sim, q_pole, th_min, th_max)
            %obj = UAVSIM(model, f_sim) Construct UAV simulator
            %   model = UAV model [UAVModel]
            %   f_sim = Sim frequency [Hz]
            %   q_pole = Quat ctrl pole [s^-1]
            %   th_min = Min thrust ratio [0-1]
            %   th_max = Max thrust ratio [0-1]
            obj = obj@UAVInterface(model);
            obj.f_sim = f_sim;
            obj.t_sim = 1 / f_sim;
            obj.q = Quat();
            obj.w = zeros(3, 1);
            obj.k_q = q_pole^2;
            obj.k_w = -2 * q_pole;
            acc_max = 4 * obj.model.f_max / obj.model.mass;
            obj.a_min = th_min * acc_max;
            obj.a_max = th_max * acc_max;
        end
        
        function [q, w, f, acc, tz, q_cmd, alp_cmd, running] = update(obj, acc_cmd, tz_cmd)
            %[q, w] = UPDATE(acc, tz) Run sim iteration and get state
            %   Inputs:
            %       acc_cmd = Global accel cmd [m/s^2]
            %       tz_cmd = Heading cmd [rad]
            %   Outputs:
            %       q = New pose [Quat]
            %       w = New angular velocity [rad/s]
            %       f = Propeller forces [N]
            %       acc = Global accel [m/s^2]
            %       tz = Heading [rad]
            %       running = [Bool]
            
            % Simulate controller
            [q_cmd, acc_mag] = obj.lap(acc_cmd, tz_cmd);
            alp_cmd = obj.qoc(q_cmd);
            f = obj.frc(alp_cmd, acc_mag);
            a = obj.model.M_mat \ f;
            
            % Compute accel and heading
            acc = [0; 0; a(4)];
            acc = obj.q.rotate(acc) - obj.model.g_vec;
            x_hat = obj.q.rotate([1; 0; 0]);
            tz = atan2(x_hat(2), x_hat(1));
            
            % Update orientation
            if norm(obj.w) > 0
                q_theta = norm(obj.w) * obj.t_sim;
                obj.q = unit(obj.q * Quat(obj.w, q_theta));
            end
            alp = a(1:3);
            obj.w = obj.w + alp * obj.t_sim;
            q = obj.q;
            w = obj.w;
            
            % Check for crash
            z_hat = [0; 0; 1];
            z_hat = obj.q.rotate(z_hat);
            running = (z_hat(3) > 0);
        end
    end
    
    methods (Access = protected)
        function [q_cmd, acc_mag] = lap(obj, acc_cmd, tz_cmd)
            %[q_cmd, am_cmd] = LAP(obj, acc_cmd, tz_cmd)
            %   Linear Acceleration Planner
            
            % Adjust for gravity
            acc_cmd = acc_cmd + obj.model.g_vec;
            
            % Acceleration limiting (TODO UPDATE DOC)
            acc_cmd(3) = clamp(acc_cmd(3), obj.a_min, obj.a_max);
            norm_xy = norm(acc_cmd(1:2));
            p = sqrt((obj.a_max^2 - acc_cmd(3)^2)) / norm_xy;
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
        end
        
        function alp_cmd = qoc(obj, q_cmd)
            %alp_cmd = QOC(obj, q_cmd)
            %   Quaternion Orientation Controller
            q_err = q_cmd \ obj.q;
            if q_err.w < 0
                % Make error < 180 deg
                q_err = -q_err; % TODO UPDATE DOC
            end
            q_err = [q_err.x; q_err.y; q_err.z];
            alp_cmd = -(obj.k_q*q_err + obj.k_w*obj.w);
        end
        
        function f = frc(obj, alp_cmd, acc_cmd)
            %f = FRC(obj, alp_cmd, acc_cmd)
            %   Force Regulator Controller
            
            % Apply angular limit
            f_alp = obj.model.M_alp * alp_cmd;
            f_acc = obj.model.M_acc * acc_cmd;
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
            
            % Apply clamp limit
            for i = 1:4
                f(i) = clamp(f(i), obj.model.f_min, obj.model.f_max);
            end
        end
    end
end