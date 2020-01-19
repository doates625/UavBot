classdef Matlab < uav.interface.sim.Sim
    %MATLAB Matlab simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        qx_pid;     % Quat x-axis ctrl [controls.PID]
        qy_pid;     % Quat y-axis ctrl [controls.PID]
        qz_pid;     % Quat z-axis ctrl [controls.PID]
        quat_sat;   % Quat PID saturation flag [logical]
    end
    
    methods (Access = public)
        function obj = Matlab(model, params)
            %obj = MATLAB(model, params)
            %   Construct Matlab simulator
            %   
            %   Inputs:
            %   - model = UAV model [uav.Model]
            %   - params = Flight params [uav.Params]
            
            % Imports
            import('uav.Params');
            import('uav.Model');
            
            % Superconstructor
            if nargin < 2, params = Params(); end
            if nargin < 1, model = Model(); end
            obj@uav.interface.sim.Sim(model, params);
            
            % Quaternion PID controllers
            obj.qx_pid = obj.make_pid(...
                obj.params.qx_kp, ...
                obj.params.qx_ki, ...
                obj.params.qx_kd);
            obj.qy_pid = obj.make_pid(...
                obj.params.qy_kp, ...
                obj.params.qy_ki, ...
                obj.params.qy_kd);
            obj.qz_pid = obj.make_pid(...
                obj.params.qz_kp, ...
                obj.params.qz_ki, ...
                obj.params.qz_kd);
            obj.quat_sat = false;
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   
            %   Inputs:
            %   - cmd = UAV command [uav.state.Cmd]
            %   
            %   Outputs:
            %   - state = UAV state [uav.state.State]
            
            % Imports
            import('uav.state.Enum');
            
            % Copy command
            obj.cmd = cmd;
            
            % Simulate controller
            if cmd.enum == Enum.Enabled
                thr_ang = obj.ang_ctrl(cmd.ang_pos);
                thr_lin = obj.lin_ctrl(cmd.thr_lin);
                thr_props = obj.anti_windup(thr_ang, thr_lin);
            else
                thr_props = zeros(4, 1);
            end
            
            % Simulate dynamics
            state = obj.update_sim(thr_props, cmd.enum);
        end
        
        function set_params(obj, params)
            %SET_PARAMS(obj, params)
            %   Set flight parameters
            %   
            %   Inputs:
            %   - params = Flight params [UAV.Params]
            obj.params = params;
            obj.qx_pid.set_k_p(params.qx_kp);
            obj.qx_pid.set_k_i(params.qx_ki);
            obj.qx_pid.set_k_d(params.qx_kd);
            obj.qy_pid.set_k_p(params.qy_kp);
            obj.qy_pid.set_k_i(params.qy_ki);
            obj.qy_pid.set_k_d(params.qy_kd);
            obj.qz_pid.set_k_p(params.qz_kp);
            obj.qz_pid.set_k_i(params.qz_ki);
            obj.qz_pid.set_k_d(params.qz_kd);
        end
    end
    
    methods (Access = protected)
        function thr_ang = ang_ctrl(obj, ang_pos_cmd)
            %thr_ang = ANG_CTRL(obj, ang_pos_cmd)
            %   Angular control system
            %   
            %   Inputs:
            %   - ang_pos_cmd = Angular position command [quat.Quat]
            %   
            %   Outputs:
            %   - thr_ang = Angular prop throttle vector [0, 1]
            
            % Compute angular error
            ang_err = pos_w(ang_pos_cmd \ obj.state.ang_pos);
            
            % Run PID controllers
            thr_ang_d = zeros(3, 1);
            thr_ang_d(1) = obj.qx_pid.update(...
                -ang_err.x, ...
                obj.params.qx_ff, ...
                obj.quat_sat);
            thr_ang_d(2) = obj.qy_pid.update(...
                -ang_err.y, ...
                obj.params.qy_ff, ...
                obj.quat_sat);
            thr_ang_d(3) = obj.qz_pid.update(...
                -ang_err.z, ...
                obj.params.qz_ff, ...
                obj.quat_sat);
            
            % Convert to prop throttles
            thr_ang = obj.model.N_ang * thr_ang_d;
        end
        
        function thr_lin = lin_ctrl(obj, thr_lin_cmd)
            %thr_lin = LIN_CTRL(obj, thr_lin_cmd)
            %   Linear control system
            %   
            %   Inputs:
            %   - thr_lin_cmd = Linear throttle cmd [0, 1]
            %   
            %   Outputs:
            %   - thr_lin = Linear prop throttle vector [0, 1]
            import('controls.clamp');
            thr_lin_d = clamp(...
                thr_lin_cmd, ...
                obj.params.thr_min, ...
                obj.params.thr_max);
            thr_lin = obj.model.N_lin * thr_lin_d;
        end
        
        function thr_props = anti_windup(obj, thr_ang, thr_lin)
            %thr_props = ANTI_WINDUP(obj, thr_ang, thr_lin)
            %   Anti-windup controller
            %   
            %   Inputs:
            %   - thr_ang = Angular prop throttles [0, 1]
            %   - thr_lin = Linear prop throttles [0, 1]
            %   
            %   Outputs:
            %   - thr_props = Combined prop throttles [N]
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
            %pid = MAKE_PID(obj, kp, ki, kd)
            %   Make quaternion PID controller
            %   
            %   Inputs:
            %   - kp = P-gain [thr/rad]
            %   - ki = I-gain [thr/(rad*s)]
            %   - kd = D-gain [thr/(rad/s)]
            %   
            %   Outputs:
            %   - pid = PID controller [controls.PID]
            import('controls.PID');
            pid = PID(kp, ki, kd, -realmax(), +realmax(), obj.model.f_ctrl);
        end
    end
end