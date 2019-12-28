classdef UAVSim < UAVInterface
    %UAVSIMULATOR Simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        f_sim   % Sim frequency [Hz]
        t_sim   % Sim period [s]
    end
    
    properties (GetAccess = protected)
        q       % Orientation [Quat]
        w       % Angular velocity [rad/s]
        k_q     % Quaternion gain [rad/s^2]
        k_w     % Velocity gain [s^-1]
        M_alp   % Angular mass matrix
        M_acc   % Linear mass matrix
    end
    
    methods (Access = public)
        function obj = UAVSim(model, f_sim, s_q)
            %obj = UAVSIM(model, f_sim) Construct UAV simulator
            %   model = UAV model [UAVModel]
            %   f_sim = Sim frequency [Hz]
            %   s_q = Quat ctrl pole [s^-1]
            obj = obj@UAVInterface(model);
            obj.f_sim = f_sim;
            obj.t_sim = 1 / f_sim;
            obj.q = Quat();
            obj.w = zeros(3, 1);
            obj.k_q = s_q^2;
            obj.k_w = -2 * s_q;
            obj.M_alp = obj.model.M(:,1:3);
            obj.M_acc = obj.model.M(:,4:4);
        end
        
        function [q, w, f, acc, tz, q_cmd] = update(obj, acc_cmd, tz_cmd)
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
            
            % Simulate controller
            tz_cmd = wrap(tz_cmd, -pi, +pi);
            [q_cmd, acc_mag] = obj.lap(acc_cmd, tz_cmd);
            alp_cmd = obj.qoc(q_cmd);
            f = obj.frc(alp_cmd, acc_mag);
            a = obj.model.M \ f;
            
            % Compute accel and heading
            acc = [0; 0; a(4)];
            acc = obj.q.rotate(acc) - obj.model.g;
            x_hat = obj.q.rotate([1; 0; 0]);
            tz = atan2(x_hat(2), x_hat(1));
            
            % Update orientation
            if norm(obj.w) > 0
                q_theta = norm(obj.w) * obj.t_sim;
                obj.q = obj.q * Quat(obj.w, q_theta);
            end
            alp = a(1:3);
            obj.w = obj.w + alp * obj.t_sim;
            q = obj.q;
            w = obj.w;
        end
    end
    
    methods (Access = protected)
        function [q_cmd, acc_mag] = lap(obj, acc_cmd, tz_cmd)
            %[q_cmd, am_cmd] = LAP(obj, acc_cmd, tz_cmd)
            %   Linear Acceleration Planner
            
            % Orientation
            acc_cmd = acc_cmd + obj.model.g;
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
            n_hat = obj.q.rotate([0; 0; 1]);
            acc_mag = acc_cmd(3) / n_hat(3);
        end
        
        function alp_cmd = qoc(obj, q_cmd)
            %alp_cmd = QOC(obj, q_cmd)
            %   Quaternion Orientation Controller
            q_err = q_cmd / obj.q;
            q_err = [q_err.x; q_err.y; q_err.z];
            alp_cmd = obj.k_q * q_err - obj.k_w * obj.w; % TODO UPDATE DOC
        end
        
        function f = frc(obj, alp_cmd, acc_cmd)
            %f = FRC(obj, alp_cmd, acc_cmd)
            %   Force Regulator Controller
            
            % Apply angular limit
            f_alp = obj.M_alp * alp_cmd;
            f_acc = obj.M_acc * acc_cmd;
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