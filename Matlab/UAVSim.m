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
            
            % Linear Acceleration Planning
            acc_cmd = acc_cmd + obj.model.g;
            am_cmd = norm(acc_cmd);
            acc_hat = acc_cmd / am_cmd;
            tz = tz_cmd;
            ty = asin(acc_hat(1) + tan(tz)*acc_hat(2));
            tx = asin(sin(tz)*acc_hat(1) - cos(tz)*acc_hat(2));
            qz = Quat([0; 0; 1], tz);
            qy = Quat([0; 1; 0], ty);
            qx = Quat([1; 0; 0], tx);
            q_cmd = qz * qy * qx;
            
            % Quaternion Orientation Control
            q_err = q_cmd / obj.q;
            q_err = [q_err.x; q_err.y; q_err.z];
            alp_cmd = obj.k_q * q_err - obj.k_w * obj.w; % TODO UPDATE DOC
            
            % Force Regulator Control
            M = obj.model.M;
            f_alp = M(:,1:3) * alp_cmd;
            f_acc = M(:,4:4) * am_cmd;
            p_min = 1;
            for i = 1:4
                if f_alp(i) > 0
                    p = (obj.model.f_max - f_acc(i)) / f_alp(i);
                elseif f_alp(i) < 0
                    p = (obj.model.f_min - f_acc(i)) / f_alp(i);
                else
                    p = 1;
                end
                if p < p_min
                    p_min = p;
                end
            end
            f = p_min * f_alp + f_acc;
            a_gen = obj.model.M \ f;

            % Update orientation
            if norm(obj.w) > 0
                q_theta = norm(obj.w) * obj.t_sim;
                obj.q = obj.q * Quat(obj.w, q_theta);
            end
            alp = a_gen(1:3);
            obj.w = obj.w + alp * obj.t_sim;
            q = obj.q;
            w = obj.w;
            
            % Compute accel and heading
            acc = [0; 0; a_gen(4)];
            acc = obj.q.rotate(acc) - obj.model.g;
            x_hat = obj.q.rotate([1; 0; 0]);
            tz = atan2(x_hat(2), x_hat(1));
        end
    end
end