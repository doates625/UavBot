classdef Xbox < UAV.CmdSrcs.CmdSrc
    %XBOX Xbox360 game controller for UAV piloting
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        xbox;       % Xbox360 controller [Xbox360]
        acc_max;    % Max acceleration cmd [m/s^2]
        wz_max;     % Max heading cmd rate [rad/s]
        tz_int;     % Angle cmd integrator [Integrator]
        state;      % State cmd [char]
        timer;      % Timer object [Timer]
        first_get;  % First get flag [logical]
    end
    
    methods (Access = public)
        function obj = Xbox(xbox_id, xbox_dz, acc_max, wz_max)
            %obj = XBOX(xbox_id, xbox_dz, acc_max, wz_max)
            %   Construct Xbox360 UAV controller
            %   Inputs:
            %       xbox_id = Joystick ID [1-4]
            %       xbox_dz = Joystick dead zone [0-1]
            %       acc_max = Max acceleration cmd [m/s^2]
            %       wz_max = Max heading cmd rate [rad/s]
            
            % Default args
            import('UAV.default_arg');
            if nargin < 4, wz_max = pi/4; end
            if nargin < 3, acc_max = 1.0; end
            if nargin < 2, xbox_dz = 0.075; end
            if nargin < 1, xbox_id = 1; end
            
            % Init properties
            obj.xbox = Xbox360(xbox_id, xbox_dz);
            obj.acc_max = acc_max;
            obj.wz_max = wz_max;
            obj.tz_int = Integrator();
            obj.state = 'Disabled';
            obj.timer = Timer();
            obj.first_get = true;
        end
        
        function [cmd, t] = get_cmd(obj)
            %[cmd, t] = GET_CMD(obj) Get commands and time
            %   Outputs:
            %       cmd = UAV command [UAV.Cmd]
            %       t = Time [s]
            
            % Parse heading command
            wz = -obj.wz_max * obj.xbox.axis('Rx');
            tz = obj.tz_int.update(wz);
            
            % Parse accel command
            acc = zeros(3, 1);
            acc(1) = -obj.acc_max * obj.xbox.axis('Ly');
            acc(2) = -obj.acc_max * obj.xbox.axis('Lx');
            acc(3) = -obj.acc_max * obj.xbox.axis('Trig');
            acc = Quat([0; 0; 1], tz).rotate(acc);
            
            % Parse state command
            if obj.xbox.btn('Start'), obj.state = 'Enabled'; end
            if obj.xbox.btn('Back'), obj.state = 'Disabled'; end
            
            % Form command
            cmd = UAV.Cmd(acc, tz, obj.state);
            
            % Get time
            if obj.first_get
                t = 0;
                obj.timer.tic();
                obj.first_get = false;
            else
                t = obj.timer.toc();
            end
        end
        
        function stop = get_stop(obj)
            %stop = GET_STOP(obj) Get stop flag [logical]
            stop = obj.xbox.btn('B');
        end
        
        function set_tz(obj, tz)
            %SET_TZ(obj, tz) Overrides heading cmd [rad]
            obj.tz_int.set(tz);
        end
    end
end