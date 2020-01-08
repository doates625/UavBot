classdef Ctrl
    %CTRL UAV control model parameters
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        s_qx;   % Quaternion x-axis pole [s^-1]
        s_qy;   % Quaternion y-axis pole [s^-1]
        s_qz;   % Quaternion z-axis pole [s^-1]
        s_az;   % Acceleration z-axis pole [s^-1]
        fr_min; % Min prop thrust ratio [N/N]
        fr_max; % Max prop thrust ratio [N/N]
    end
    
    methods (Access = public)
        function obj = Ctrl(s_qx, s_qy, s_qz, s_az, fr_min, fr_max) 
            %CTRL Construct control model
            %   
            %   obj = CTRL(s_qx, s_qy, s_qz, s_az, fr_min, fr_max)
            %       Construct control model with custom params
            %       Inputs:
            %           s_qx = Quaternion x-axis pole [s^-1]
            %           s_qy = Quaternion y-axis pole [s^-1]
            %           s_qz = Quaternion z-axis pole [s^-1]
            %           s_az = Acceleration z-axis pole [s^-1]
            %           fr_min = Min prop thrust ratio [N/N]
            %           fr_max = Max prop thrust ratio [N/N]
            %   
            %   obj = CTRL()
            %       Construct control model with default params
            
            % Default args
            if nargin == 0
                s_qx = -3.0;
                s_qy = -3.0;
                s_qz = -3.0;
                s_az = -3.0;
                fr_min = 0.1;
                fr_max = 0.9;
            elseif nargin ~= 6
                error('Invalid nargin.')
            end
            
            % Copy params
            obj.s_qx = s_qx;
            obj.s_qy = s_qy;
            obj.s_qz = s_qz;
            obj.s_az = s_az;
            obj.fr_min = fr_min;
            obj.fr_max = fr_max;
        end
    end
end