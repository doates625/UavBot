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
                s_qx = -8.0;
                s_qy = -8.0;
                s_qz = -8.0;
                s_az = -10.0;
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
        
        function print_cpp(obj)
            %PRINT_CPP(obj) Prints C++ code for constants
            clc
            fprintf('\n// Control Constants\n')
            f_sim = UAV.Interfaces.Sims.Sim.f_sim;
            fprintf('const float f_ctrl = %.1ff;\t\t// Control freq [Hz]\n', f_sim);
            fprintf('const float s_qx = %+.1ff;\t\t// Quat x-axis pole [s^-1]\n', obj.s_qx);
            fprintf('const float s_qy = %+.1ff;\t\t// Quat y-axis pole [s^-1]\n', obj.s_qy);
            fprintf('const float s_qz = %+.1ff;\t\t// Quat z-axis pole [s^-1]\n', obj.s_qz);
            fprintf('const float s_az = %+.1ff;\t\t// Accel z-axis pole [s^-1]\n', obj.s_az);
            fprintf('const float fr_min = %.1ff;\t\t// Min prop thrust ratio [N/N]\n', obj.fr_min)
            fprintf('const float fr_max = %.1ff;\t\t// Max prop thrust ratio [N/N]\n', obj.fr_max)
            fprintf('const float tau_min = -1e10f;\t// PID torque min [N*m]\n')
            fprintf('const float tau_max = +1e10f;\t// PID torque max [N*m]\n')
            fprintf('\n')
        end
    end
end