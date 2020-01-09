classdef Xbox < UAV.CmdSrcs.CmdSrc
    %XBOX Xbox game controller for UAV piloting
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        xbox;       % Xbox controller [Xbox360]
        acc_max;    % Max acceleration cmd [m/s^2]
        wz_max;     % Max heading cmd rate [rad/s]
        tz_int;     % Angle cmd integrator [Integrator]
        state_enum; % State machine enum cmd [UAV.State.Enum]
        timer;      % Timer object [Timer]
        first_get;  % First get flag [logical]
    end
    
    methods (Access = public)
        function obj = Xbox(xbox_id, xbox_dz, acc_max, wz_max)
            %obj = XBOX(xbox_id, xbox_dz, acc_max, wz_max)
            %   Construct Xbox UAV controller
            %   Inputs:
            %       xbox_id = Joystick ID [1-4]
            %       xbox_dz = Joystick dead zone [0-1]
            %       acc_max = Max acceleration cmd [m/s^2]
            %       wz_max = Max heading cmd rate [rad/s]
            
            % Default args
            if nargin < 4, wz_max = pi/4; end
            if nargin < 3, acc_max = 1.0; end
            if nargin < 2, xbox_dz = 0.075; end
            if nargin < 1, xbox_id = 1; end
            
            % Init properties
            import('UAV.State.Enum');
            obj.xbox = Xbox360(xbox_id, xbox_dz);
            obj.acc_max = acc_max;
            obj.wz_max = wz_max;
            obj.tz_int = Integrator();
            obj.state_enum = Enum.Disabled;
            obj.timer = Timer();
            obj.first_get = true;
        end
        
        function [cmd, time] = get_cmd(obj)
            %[cmd, time] = GET_CMD(obj) Get commands and time
            %   Outputs:
            %       cmd = UAV command [UAV.State.Cmd]
            %       time = Time [s]
            
            % Parse heading command
            wz = -obj.wz_max * obj.xbox.axis('Rx');
            ang_z = obj.tz_int.update(wz);
            
            % Parse linear acceleration command
            lin_acc = zeros(3, 1);
            lin_acc(1) = -obj.acc_max * obj.xbox.axis('Ly');
            lin_acc(2) = -obj.acc_max * obj.xbox.axis('Lx');
            lin_acc(3) = -obj.acc_max * obj.xbox.axis('Trig');
            lin_acc = Quat([0; 0; 1], ang_z).rotate(lin_acc);
            
            % Parse state command
            import('UAV.State.Enum');
            if obj.xbox.btn('B')
                obj.state_enum = Enum.Disabled;
            elseif obj.xbox.btn('Start')
                if obj.state_enum ~= Enum.Enabled
                    obj.tz_int.set(0);
                end
                obj.state_enum = Enum.Enabled;
            end
            
            % Form command
            cmd = UAV.State.Cmd(lin_acc, ang_z, obj.state_enum);
            
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