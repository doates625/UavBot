classdef Phys < handle
    %PHYS UAV physical model parameters
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Constant)
        % Gravity [m/s^2]
        g_sca = 9.807;
        g_vec = [0; 0; UAV.PhysModel.g_sca];
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
        M_ang;      % Angular mass matrix [kg*m^2]
        M_lin;      % Linear mass matrix [kg]
        M_bar_ang;  % Inverse angular mass matrix [(kg*m^2)^-1]
        M_bar_lin;  % Inverse linear mass matrix [kg^-1]
        D_bar_ang;  % Inverse moment arm matrix [m^-1]
    end
    
    methods (Access = public)
        function obj = Phys(...
                I_xx, I_yy, I_zz, r_x, r_y, r_z, mass, f_min, f_max)
            %PHYS Construct physical model
            %   
            %   obj = PHYS(...
            %       I_xx, I_yy, I_zz, r_x, r_y, r_z, mass, f_min, f_max)
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
                import('UAV.default_arg');
                I_xx = default_arg('I_xx');
                I_yy = default_arg('I_yy');
                I_zz = default_arg('I_zz');
                r_x = default_arg('r_x');
                r_y = default_arg('r_y');
                r_z = default_arg('r_z');
                mass = default_arg('mass');
                f_min = default_arg('f_min');
                f_max = default_arg('f_max');
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
            D_mat = [...
                +rx, -rx, +rx, -rx; ...
                -ry, -ry, +ry, +ry; ...
                +rz, -rz, -rz, +rz; ...
                  1,   1,   1,   1];
            M_sig = diag([I_xx, I_yy, I_zz, mass]);
            M_mat = D_mat \ M_sig;
            obj.M_ang = M_mat(:, 1:3);
            obj.M_lin = M_mat(:, 4:4);
            M_bar = inv(M_mat);
            obj.M_bar_ang = M_bar(1:3, :);
            obj.M_bar_lin = M_bar(4:4, :);
            D_bar = inv(D_mat);
            obj.D_bar_ang = D_bar(:, 1:3);
        end
    end
end