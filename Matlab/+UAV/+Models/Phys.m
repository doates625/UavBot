classdef Phys < handle
    %PHYS UAV physical model parameters
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Constant)
        % Gravity [m/s^2]
        g_sca = 9.807;
        g_vec = [0; 0; UAV.PhysModel.g_sca];
    end
    
    properties (SetAccess = protected)
        I_xx;   % Inertia x-axis [kg*m^2]
        I_yy;   % Inertia y-axis [kg*m^2]
        I_zz;   % Inertia z-axis [kg*m^2]
        r_x;    % Moment arm x-axis [m]
        r_y;    % Moment arm y-axis [m]
        r_z;    % Moment arm z-axis [m]
        mass;   % Total mass [kg]
        f_min;  % Min prop force [N]
        f_max;  % Max prop force [N]
        D_bar;  % Inv moment arm matrix [m^-1]
    end
    
    methods
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
            obj.D_bar = 0.25 * [...
                +1/r_x, -1/r_y, +1/r_z; ...
                -1/r_x, -1/r_y, -1/r_z; ...
                +1/r_x, +1/r_y, -1/r_z; ...
                -1/r_x, +1/r_y, +1/r_z];
        end
    end
end