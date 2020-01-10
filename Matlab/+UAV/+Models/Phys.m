classdef Phys < handle
    %PHYS UAV physical model parameters
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Constant)
        % Gravity [m/s^2]
        g_sca = 9.807;
        g_vec = [0; 0; UAV.Models.Phys.g_sca];
    end
    
    properties (SetAccess = protected)
        I_xx;       % Inertia x-axis [kg*m^2]
        I_yy;       % Inertia y-axis [kg*m^2]
        I_zz;       % Inertia z-axis [kg*m^2]
        r_x;        % Moment arm x-axis [m]
        r_y;        % Moment arm y-axis [m]
        r_z;        % Moment arm z-axis [m]
        mass;       % Total mass [kg]
        f_min;      % Min prop force [N]
        f_max;      % Max prop force [N]
        M_bar_ang;  % Inv angular masses [(kg*m^2)^-1]
        M_bar_lin;  % Inv linear masses [kg^-1]
        D_mat;      % Generalized moment arm matrix
        D_mat_ang;  % Angular moment arms [m]
        D_mat_lin;  % Linear moment arms [N/N]
        D_bar_ang;  % Inv angular moment arms [m^-1]
        D_bar_lin;  % Inv linear moment arms [N/N]
    end
    
    methods (Access = public)
        function obj = Phys(I_xx, I_yy, I_zz, r_x, r_y, r_z, mass, f_min, f_max)
            %PHYS Construct physical model
            %   
            %   obj = PHYS(I_xx, I_yy, I_zz, r_x, r_y, r_z, mass, f_min, f_max)
            %       Construct physical model with custom params
            %       Inputs:
            %           I_xx = Inertia x-axis [kg*m^2]
            %           I_yy = Inertia y-axis [kg*m^2]
            %           I_zz = Inertia z-axis [kg*m^2]
            %           r_x = Moment arm x-axis [m]
            %           r_y = Moment arm y-axis [m]
            %           r_z = Moment arm z-axis [m]
            %           mass = Total mass [kg]
            %           f_min = Min prop force [N]
            %           f_max = Max prop force [N]
            %   
            %   obj = PHYS()
            %       Construct physical model with default params

            % Default args
            if nargin == 0
                I_xx = 1.15e-03;
                I_yy = 1.32e-03;
                I_zz = 2.24e-03;
                r_x = 9.30e-02;
                r_y = 9.30e-02;
                r_z = 5.50e-02;
                mass = 0.546;
                f_min = 0.00;
                f_max = 2.46;
            elseif nargin ~= 9
                error('Invalid nargin.')
            end
            
            % Copy params
            obj.I_xx = I_xx;
            obj.I_yy = I_yy;
            obj.I_zz = I_zz;
            obj.r_x = r_x;
            obj.r_y = r_y;
            obj.r_z = r_z;
            obj.mass = mass;
            obj.f_min = f_min;
            obj.f_max = f_max;
            
            % Derived params
            obj.D_mat = [...
                +r_x, -r_x, +r_x, -r_x; ...
                -r_y, -r_y, +r_y, +r_y; ...
                +r_z, -r_z, -r_z, +r_z; ...
                   1,    1,    1,    1];
            obj.D_mat_ang = obj.D_mat(1:3, :);
            obj.D_mat_lin = obj.D_mat(4:4, :);
            M_sig = diag([I_xx, I_yy, I_zz, mass]);
            M_mat = obj.D_mat \ M_sig;
            M_bar = inv(M_mat);
            obj.M_bar_ang = M_bar(1:3, :);
            obj.M_bar_lin = M_bar(4:4, :);
            D_bar = inv(obj.D_mat);
            obj.D_bar_ang = D_bar(:, 1:3);
            obj.D_bar_lin = D_bar(:, 4:4);
        end
        
        function print_cpp(obj)
            %PRINT_CPP(obj) Prints C++ code for constants
            clc
            
            % Physical constants
            fprintf('\n// Physical Constants\n');
            fprintf('const float I_xx = %.2ef;\t// Inertia x-axis [kg*m^2]\n', obj.I_xx);
            fprintf('const float I_yy = %.2ef;\t// Inertia y-axis [kg*m^2]\n', obj.I_yy);
            fprintf('const float I_zz = %.2ef;\t// Inertia z-axis [kg*m^2]\n', obj.I_zz);
            fprintf('const float mass = %.3ff;\t\t// Total mass [kg]\n', obj.mass);
            fprintf('const float gravity = %.3ff;\t// Gravity [m/s^2]\n', obj.g_sca);
            fprintf('\n');
            
            % Angular moment arm
            fprintf('// Inverse Angular Moment Arm Matrix\n');
            print_mat_cpp('D_bar_ang', obj.D_bar_ang);
            fprintf('\n');
            
            % Linear moment arm
            fprintf('// Inverse Linear Moment Arm Matrix\n');
            print_mat_cpp('D_bar_lin', obj.D_bar_lin);
            fprintf('\n');
        end
    end
end