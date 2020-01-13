classdef Matlab < UAV.Interfaces.Sims.Sim
    %MATLAB Matlab simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        qx_pid;     % Quat x-axis ctrl [PID]
        qy_pid;     % Quat y-axis ctrl [PID]
        qz_pid;     % Quat z-axis ctrl [PID]
        quat_sat;   % Quat PID saturation flag [logical]
    end
    
    methods (Access = public)
        function obj = Matlab(model)
            %obj = MATLAB(model) Construct Matlab simulator with given model [UAV.Model]
            
            % Superconstructor
            if nargin < 1, model = UAV.Model(); end
            obj = obj@UAV.Interfaces.Sims.Sim(model);
            
            % Quaternion PID controllers
            obj.qx_pid = obj.make_pid(obj.model.qx_kp, obj.model.qx_ki, obj.model.qx_kd);
            obj.qy_pid = obj.make_pid(obj.model.qy_kp, obj.model.qy_ki, obj.model.qy_kd);
            obj.qz_pid = obj.make_pid(obj.model.qz_kp, obj.model.qz_ki, obj.model.qz_kd);
            obj.quat_sat = false;
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
                thr_ang = obj.ang_ctrl(cmd.ang_pos);
                thr_lin = obj.lin_ctrl(cmd.thr_lin);
                thr_props = obj.anti_windup(thr_ang, thr_lin);
            else
                thr_props = zeros(4, 1);
            end
            
            % Simulate dynamics
            state = obj.update_sim(thr_props, cmd.enum);
        end
    end
    
    methods (Access = protected)
        function thr_ang = ang_ctrl(obj, ang_pos_cmd)
            %thr_ang = ANG_CTRL(obj, ang_pos_cmd)
            %   Angular control system
            %   Inputs:
            %       ang_pos_cmd = Angular position command [Quat]
            %   Outputs:
            %       thr_ang = Angular prop throttle vector [0, 1]
            
            % Compute angular error
            ang_err = pos_w(ang_pos_cmd \ obj.state.ang_pos);
            
            % Run PID controllers
            thr_ang_d = zeros(3, 1);
            thr_ang_d(1) = obj.qx_pid.update(-ang_err.x, obj.model.qx_ff, obj.quat_sat);
            thr_ang_d(2) = obj.qy_pid.update(-ang_err.y, obj.model.qy_ff, obj.quat_sat);
            thr_ang_d(3) = obj.qz_pid.update(-ang_err.z, obj.model.qz_ff, obj.quat_sat);
            
            % Convert to prop throttles
            thr_ang = obj.model.N_ang * thr_ang_d;
        end
        
        function thr_lin = lin_ctrl(obj, thr_lin_cmd)
            %thr_lin = LIN_CTRL(obj, thr_lin_cmd)
            %   Linear control system
            %   Inputs:
            %       thr_lin_cmd = Linear throttle cmd [0, 1]
            %   Outputs:
            %       thr_lin = Linear prop throttle vector [0, 1]
            thr_lin_d = clamp(thr_lin_cmd, obj.model.thr_min, obj.model.thr_max);
            thr_lin = obj.model.N_lin * thr_lin_d;
        end
        
        function thr_props = anti_windup(obj, thr_ang, thr_lin)
            %thr_props = ANTI_WINDUP(obj, thr_ang, thr_lin)
            %   Anti-windup controller
            %   Limits angular throttles and set flag to prevent windup
            %   Inputs:
            %       thr_ang = Angular prop throttles [0, 1]
            %       thr_lin = Linear prop throttles [0, 1]
            %   Outputs:
            %       thr_props = Combined prop throttles [N]
            p_min = 1;
            obj.quat_sat = false;
            for i = 1:4
                if thr_ang(i) > 0
                    p = (1 - thr_lin(i)) / thr_ang(i);
                elseif thr_ang(i) < 0
                    p = (0 - thr_lin(i)) / thr_ang(i);
                else
                    p = 1;
                end
                if 0 < p && p < p_min
                    p_min = p;
                    obj.quat_sat = true;
                end
            end
            thr_props = p_min * thr_ang + thr_lin;
        end
        
        function pid = make_pid(obj, kp, ki, kd)
            %pid = MAKE_PID(obj, kp, ki, kd) Make quaternion PID controller
            %   kp = P-gain [thr/rad]
            %   ki = I-gain [thr/(rad*s)]
            %   kd = D-gain [thr/(rad/s)]
            pid = PID(kp, ki, kd, -realmax(), +realmax(), obj.model.f_ctrl);
        end
    end
end