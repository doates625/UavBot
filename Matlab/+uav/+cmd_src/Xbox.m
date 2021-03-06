classdef Xbox < uav.cmd_src.CmdSrc
    %XBOX Xbox game controller for UAV piloting
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        vel_z_max;  % Max yaw rate [rad/s]
        ang_y_max;  % Max pitch angle [rad]
        ang_x_max;  % Max roll angle [rad]
    end
    
    properties (Access = protected)
        xbox;       % Xbox controller [Xbox360]
        ang_z_int;  % Yaw cmd integrator [Integrator]
        enum_cmd;   % State machine enum cmd [UAV.State.Enum]
        timer;      % Timer object [Timer]
        first_get;  % First get flag [logical]
    end
    
    methods (Access = public)
        function obj = Xbox(uav, vel_z_max, ang_y_max, ang_x_max)
            %obj = XBOX(uav, vel_z_max, ang_y_max, ang_x_max)
            %   Construct Xbox UAV controller
            %   
            %   Inputs:
            %   - uav = UAV interface [uav.interface.Interface]
            %   - vel_z_max = Max yaw rate [rad/s]
            %   - ang_y_max = Max pitch angle [rad]
            %   - ang_x_max = Max roll angle [rad]
            
            % Imports
            import('uav.Params');
            import('uav.Model');
            import('uav.state.Enum');
            import('game_ctrl.Xbox360');
            import('controls.Integrator');
            import('timing.Timer');
            
            % Default args
            if nargin < 4, ang_x_max = deg2rad(20); end
            if nargin < 3, ang_y_max = deg2rad(20); end
            if nargin < 2, vel_z_max = pi/4; end
            
            % Init properties
            obj@uav.cmd_src.CmdSrc(uav);
            obj.vel_z_max = vel_z_max;
            obj.ang_y_max = ang_y_max;
            obj.ang_x_max = ang_x_max;
            obj.xbox = Xbox360();
            obj.xbox.set_dz('LX', 0.08);
            obj.xbox.set_dz('LY', 0.08);
            obj.xbox.set_dz('RX', 0.05);
            obj.ang_z_int = Integrator();
            obj.enum_cmd = Enum.Disabled;
            obj.timer = Timer();
            obj.first_get = true;
        end
        
        function [cmd, time] = get_cmd(obj)
            %[cmd, time] = GET_CMD(obj) Get commands and time
            %   
            %   Outputs:
            %   - cmd = UAV command [uav.state.Cmd]
            %   - time = Time [s]
            
            % Imports
            import('uav.state.Enum');
            import('uav.state.Cmd');
            import('controls.clamp');
            
            % Parse orientation cmd
            vel_z = obj.vel_z_max * -obj.xbox.axis('RX');
            ang_z = obj.ang_z_int.update(vel_z);
            ang_y = obj.ang_y_max * -obj.xbox.axis('LY');
            ang_x = obj.ang_x_max * +obj.xbox.axis('LX');
            ang_pos = obj.eul_to_quat(ang_z, ang_y, ang_x);
           
            % Parse linear throttle cmd
            thr_min = obj.uav.params.thr_min;
            thr_max = obj.uav.params.thr_max;
            thr_rng = thr_max - thr_min;
            thr_lin = thr_min + thr_rng * -obj.xbox.axis('Trig');
            thr_lin = clamp(thr_lin, thr_min, thr_max);
            
            % Parse state command
            if obj.xbox.btn('B') || obj.get_stop()
                obj.enum_cmd = Enum.Disabled;
            elseif obj.xbox.btn('Start')
                if obj.enum_cmd ~= Enum.Enabled
                    obj.ang_z_int.set(0);
                end
                obj.enum_cmd = Enum.Enabled;
            end
            
            % Form command
            cmd = Cmd(ang_pos, thr_lin, obj.enum_cmd);
            
            % Get time
            if obj.first_get
                obj.timer.tic();
                obj.first_get = false;
            end
            time = obj.timer.toc();
        end
        
        function stop = get_stop(obj)
            %stop = GET_STOP(obj) Get stop flag [logical]
            stop = obj.xbox.btn('Back');
        end
    end
end