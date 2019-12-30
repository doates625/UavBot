classdef (Abstract) UavInterface < handle
    %UAVINTERFACE Class for UAV interfaces (real and simulated)
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        q;  % Orientation [Quat]
        w;  % Angular vel [rad/s]
    end
    
    methods (Access = public)
        function obj = UavInterface()
            %obj = UAVINTERFACE() Constructor
            %   Orientation set to unity
            %   Angular velocity set to 0
            obj.q = Quat();
            obj.w = zeros(3, 1);
        end
    end
    
    methods (Access = public, Abstract)
        [q, w, acc, tz, f, stat] = update(obj, acc_cmd, tz_cmd);
        %[q, w, acc, tz, f, stat] = UPDATE(obj, acc_cmd, tz_cmd)
        %   Run simulation iteration and get states
        %   
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
    end
    
    methods (Access = protected)
         function [tz, stat] = proc_quat(obj)
            %[tz, stat] = PROC_QUAT(obj)
            %   Process orientation
            %   Outputs:
            %       tz = Heading [rad]
            %       stat Status [0 = OK, 1 = failed]
            
            % Compute heading
            x_hat = obj.q.rotate([1; 0; 0]);
            tz = atan2(x_hat(2), x_hat(1));
            
            % Compute status
            z_hat = [0; 0; 1];
            z_hat = obj.q.rotate(z_hat);
            stat = (z_hat(3) < 0);
        end
    end
end