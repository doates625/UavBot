classdef Model < handle
    %MODEL UAV physical and control constants
    %   Shorthand notation:
    %       inr = Inertia [kg*m^2]
    %       rad = Radius [m]
    %       f = Force [N]
    %       q = Quaternion
    %       prop = Propeller
    %       mat = Matrix
    %       inv = Inverted matrix
    %       ang = Angular [rad]
    %       lin = Linear [m]
    %       acc = Acceleration
    %       adj = Adjustment
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (GetAccess = public, Constant)
        gravity_sca = 9.807;
        gravity_vec = [0; 0; UAV.Model.gravity_sca];
    end
    
    properties (SetAccess = protected)
        % Physical Constants
        inr_xx;     % Inertia moment x [kg*m^2]
        inr_yy;     % Inertia moment y [kg*m^2]
        inr_zz;     % Inertia moment z [kg*m^2]
        mass;       % Mass [kg]
        rad_x;      % Prop moment arm x [m]
        rad_y;      % Prop moment arm y [m]
        rad_z;      % Prop moment arm z [m]
        f_prop_min; % Min single prop force [N]
        f_prop_max; % Max single prop force [N]
        
        % Control Constants
        f_rat_min;  % Min lin thrust ratio [N/N]
        f_rat_max;  % Max lin thrust ratio [N/N]
        pole_qx;    % Quat pole x [s^-1]
        pole_qy;    % Quat pole y [s^-1]
        pole_qz;    % Quat pole z [s^-1]
        pole_az;    % Lin acc pole z [s^-1]
        qx_kp_adj;  % Quat-x P-gain fractional adj
        qx_ki_adj;  % Quat-x I-gain fractional adj
        qx_kd_adj;  % Quat-x D-gain fractional adj
        qy_kp_adj;  % Quat-y P-gain fractional adj
        qy_ki_adj;  % Quat-y I-gain fractional adj
        qy_kd_adj;  % Quat-y D-gain fractional adj
        qz_kp_adj;  % Quat-z P-gain fractional adj
        qz_ki_adj;  % Quat-z I-gain fractional adj
        qz_kd_adj;  % Quat-z D-gain fractional adj
        
        % Derived Constants
        M_mat_ang;  % Mass angular [kg*m^2]
        M_mat_lin;  % Mass linear [kg]
        M_inv_ang;  % Mass angular inverse [(kg*m^2)^-1]
        M_inv_lin;  % Mass linear inverse [kg^-1]
        D_mat_ang;  % Moment arm angular [m]
        D_mat_lin;  % Moment arm linear []
        D_inv_ang;  % Moment arm angular inverse [m^-1]
        D_inv_lin;  % Moment arm linear inverse []
        qx_kp;      % Quat-x P-gain [N*m/rad]
        qx_ki;      % Quat-x I-gain [N*m/(rad*s)]
        qx_kd;      % Quat-x D-gain [N*m/(rad/s)]
        qy_kp;      % Quat-y P-gain [N*m/rad]
        qy_ki;      % Quat-y I-gain [N*m/(rad*s)]
        qy_kd;      % Quat-y D-gain [N*m/(rad/s)]
        qz_kp;      % Quat-z P-gain [N*m/rad]
        qz_ki;      % Quat-z I-gain [N*m/(rad*s)]
        qz_kd;      % Quat-z D-gain [N*m/(rad/s)]
        az_kp;      % Acc-z P-gain [N/(m/s^2)]
        az_ki;      % Acc-z I-gain [N/(m/s^1)]
        az_kd;      % Acc-z D-gain [N/(m/s^3)]
    end
    
    methods
        function obj = Model()
            %MODEL Construct UAV model
            %   obj = MODEL(...
            %       inr_xx, inr_yy, inr_zz, mass, ...
            %       rad_x, rad_y, rad_z, ...
            %       f_prop_min, f_prop_max, ...
            %       f_rat_min, f_rat_max, ...
            %       pole_qx, pole_qy, pole_qz, pole_az, ...
            %       qx_kp_adj, qx_ki_adj, qx_kd_adj, ...
            %       qy_kp_adj, qy_ki_adj, qy_kd_adj, ...
            %       qz_kp_adj, qz_ki_adj, qz_kd_adj)
            %       Construct custom UAV model
            %       Inputs:
            %           inr_xx = Inertia moment x [kg*m^2]
            %           inr_yy = Inertia moment y [kg*m^2]
            %           inr_zz = Inertia moment z [kg*m^2]
            %           mass = Mass [kg]
            %           rad_x = Prop moment arm x [m]
            %           rad_y = Prop moment arm y [m]
            %           rad_z = Prop moment arm z [m]
            %           f_prop_min = Min single prop force [N]
            %           f_prop_max = Max single prop force [N]
            %           f_rat_min = Min lin thrust ratio [N/N]
            %           f_rat_max = Max lin thrust ratio [N/N]
            %           pole_qx = Quat pole x [s^-1]
            %           pole_qy = Quat pole y [s^-1]
            %           pole_qz = Quat pole z [s^-1]
            %           pole_az = Lin acc pole z [s^-1]
            %           qx_kp_adj = Quat-x P-gain fractional adj
            %           qx_ki_adj = Quat-x I-gain fractional adj
            %           qx_kd_adj = Quat-x D-gain fractional adj
            %           qy_kp_adj = Quat-y P-gain fractional adj
            %           qy_ki_adj = Quat-y I-gain fractional adj
            %           qy_kd_adj = Quat-y D-gain fractional adj
            %           qz_kp_adj = Quat-z P-gain fractional adj
            %           qz_ki_adj = Quat-z I-gain fractional adj
            %           qz_kd_adj = Quat-z D-gain fractional adj
            %   obj = MODEL() Construct default model
            
            % Default args
            if nargin == 0
                inr_xx = 1.15e-03;
                inr_yy = 1.32e-03;
                inr_zz = 2.24e-03;
                mass = 0.546;
                rad_x = 9.30e-02;
                rad_y = 9.30e-02;
                rad_z = 5.50e-02;
                f_prop_min = 0.00;
                f_prop_max = 2.46;
                f_rat_min = 0.1;
                f_rat_max = 0.9;
                pole_qx = -5.0;
                pole_qy = -5.0;
                pole_qz = -3.0;
                pole_az = -8.0;
                qx_kp_adj = +0.0;
                qx_ki_adj = +0.0;
                qx_kd_adj = +0.0;
                qy_kp_adj = +0.0;
                qy_ki_adj = +0.0;
                qy_kd_adj = +0.0;
                qz_kp_adj = +0.0;
                qz_ki_adj = +0.0;
                qz_kd_adj = +0.0;
            elseif nargin ~= 24
                error('Invalid nargin.')
            end
            
            % Copy physical constants
            obj.inr_xx = inr_xx;
            obj.inr_yy = inr_yy;
            obj.inr_zz = inr_zz;
            obj.mass = mass;
            obj.rad_x = rad_x;
            obj.rad_y = rad_y;
            obj.rad_z = rad_z;
            obj.f_prop_min = f_prop_min;
            obj.f_prop_max = f_prop_max;
        
            % Copy control constants
            obj.f_rat_min = f_rat_min;
            obj.f_rat_max = f_rat_max;
            obj.pole_qx = pole_qx;
            obj.pole_qy = pole_qy;
            obj.pole_qz = pole_qz;
            obj.pole_az = pole_az;
            obj.qx_kp_adj = qx_kp_adj;
            obj.qx_ki_adj = qx_ki_adj;
            obj.qx_kd_adj = qx_kd_adj;
            obj.qy_kp_adj = qy_kp_adj;
            obj.qy_ki_adj = qy_ki_adj;
            obj.qy_kd_adj = qy_kd_adj;
            obj.qz_kp_adj = qz_kp_adj;
            obj.qz_ki_adj = qz_ki_adj;
            obj.qz_kd_adj = qz_kd_adj;
            
            % Moment arm matrices
            D_mat = zeros(4);
            D_mat(1, :) = rad_x * [+1, -1, +1, -1];
            D_mat(2, :) = rad_y * [-1, -1, +1, +1];
            D_mat(3, :) = rad_z * [+1, -1, -1, +1];
            D_mat(4, :) = ones(1, 4);
            obj.D_mat_ang = D_mat(1:3, :);
            obj.D_mat_lin = D_mat(4:4, :);
            D_inv = inv(D_mat);
            obj.D_inv_ang = D_inv(:, 1:3);
            obj.D_inv_lin = D_inv(:, 4:4);
            
            % Mass matrices
            M_sig = diag([inr_xx, inr_yy, inr_zz, mass]);
            M_mat = D_mat \ M_sig;
            obj.M_mat_ang = M_mat(:, 1:3);
            obj.M_mat_lin = M_mat(:, 4:4);
            M_inv = inv(M_mat);
            obj.M_inv_ang = M_inv(1:3, :);
            obj.M_inv_lin = M_inv(4:4, :);
            
            % PID gains
            obj.qx_kp = (+6 * inr_xx * pole_qx^2) * (1 + qx_kp_adj);
            obj.qx_ki = (-2 * inr_xx * pole_qx^3) * (1 + qx_ki_adj);
            obj.qx_kd = (-6 * inr_xx * pole_qx^1) * (1 + qx_kd_adj);
            obj.qy_kp = (+6 * inr_yy * pole_qy^2) * (1 + qy_kp_adj);
            obj.qy_ki = (-2 * inr_yy * pole_qy^3) * (1 + qy_ki_adj);
            obj.qy_kd = (-6 * inr_yy * pole_qy^1) * (1 + qy_kd_adj);
            obj.qz_kp = (+6 * inr_zz * pole_qz^2) * (1 + qz_kp_adj);
            obj.qz_ki = (-2 * inr_zz * pole_qz^3) * (1 + qz_ki_adj);
            obj.qz_kd = (-6 * inr_zz * pole_qz^1) * (1 + qz_kd_adj);
            obj.az_kp = 0;
            obj.az_ki = -mass * pole_az;
            obj.az_kd = 0;
        end
        
        function print_cpp(obj)
            %PRINT_CPP(obj) Prints C++ code for constants
            clc
            fprintf('UAV Model C++ Code:\n\n')
            fprintf('// Physical Constants\n');
            fprintf('const float inr_xx = %.2ef;\t// Inertia x-axis [kg*m^2]\n', obj.inr_xx);
            fprintf('const float inr_yy = %.2ef;\t// Inertia y-axis [kg*m^2]\n', obj.inr_yy);
            fprintf('const float inr_zz = %.2ef;\t// Inertia z-axis [kg*m^2]\n', obj.inr_zz);
            fprintf('const float mass = %.3ff;\t\t// Total mass [kg]\n', obj.mass);
            fprintf('const float gravity = %.3ff;\t// Gravity [m/s^2]\n', obj.gravity_sca);
            fprintf('\n');
            fprintf('// Control Constants\n');
            fprintf('const float f_ctrl = %.1ff;\t\t// Control freq [Hz]\n', 50.0);
            fprintf('const float f_rat_min = %.2f;\t// Min prop thrust ratio\n', obj.f_rat_min);
            fprintf('const float f_rat_max = %.2f;\t// Max prop thrust ratio\n', obj.f_rat_max);
            fprintf('const float pole_az = %+.1ff;\t// Accel z-axis pole [s^-1]\n', obj.pole_az);
            fprintf('\n');
            fprintf('// Quat X-axis Control\n');
            fprintf('const float pole_qx = %+.1ff;\t// Triple pole [s^-1]\n', obj.pole_qx);
            fprintf('const float qx_kp_adj = %+.2ff;\t// P-gain fractional adj\n', obj.qx_kp_adj);
            fprintf('const float qx_ki_adj = %+.2ff;\t// I-gain fractional adj\n', obj.qx_ki_adj);
            fprintf('const float qx_kd_adj = %+.2ff;\t// D-gain fractional adj\n', obj.qx_kd_adj);
            fprintf('\n');
            fprintf('// Quat Y-axis Control\n');
            fprintf('const float pole_qy = %+.1ff;\t// Triple pole [s^-1]\n', obj.pole_qy);
            fprintf('const float qy_kp_adj = %+.2ff;\t// P-gain fractional adj\n', obj.qy_kp_adj);
            fprintf('const float qy_ki_adj = %+.2ff;\t// I-gain fractional adj\n', obj.qy_ki_adj);
            fprintf('const float qy_kd_adj = %+.2ff;\t// D-gain fractional adj\n', obj.qy_kd_adj);
            fprintf('\n');
            fprintf('// Quat Z-axis Control\n');
            fprintf('const float pole_qz = %+.1ff;\t// Triple pole [s^-1]\n', obj.pole_qz);
            fprintf('const float qz_kp_adj = %+.2ff;\t// P-gain fractional adj\n', obj.qz_kp_adj);
            fprintf('const float qz_ki_adj = %+.2ff;\t// I-gain fractional adj\n', obj.qz_ki_adj);
            fprintf('const float qz_kd_adj = %+.2ff;\t// D-gain fractional adj\n', obj.qz_kd_adj);
            fprintf('\n');
        end
    end
end

