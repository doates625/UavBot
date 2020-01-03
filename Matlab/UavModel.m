classdef UavModel < handle
    %UAVMODEL Container for UAV model parameters
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Constant)
        % Gravity [m/s^2]
        g_sca = 9.807;
        g_vec = [0; 0; UavModel.g_sca]; 
    end
    
    properties (SetAccess = protected)
        I_xx;   % Inertia xx [kg*m^2]
        I_yy;   % IInertia yy [kg*m^2]
        I_zz;   % Inertia zz [kg*m^2]
        I_xy;   % Inertia xy [kg*m^2]
        I_xz;   % Inertia xz [kg*m^2]
        I_yz;   % Inertia yz [kg*m^2]
        r_xp;   % Moment arm x+ [m]
        r_xn;   % Moment arm x- [m]
        r_yp;   % Moment arm y+ [m]
        r_yn;   % Moment arm y- [m]
        r_z;    % Moment arm z [m]
        mass;   % Mass [kg]
        f_min;  % Min prop force [N]
        f_max;  % Max prop force [N]
        I_mat;  % Inertia matrix
        M_mat;  % Generalized mass matrix
        M_alp;  % Angular mass matrix
        M_acc;  % Linear mass matrix
    end
    
    methods (Access = public)
        function obj = UavModel(...
                I_xx, I_yy, I_zz, I_xy, I_xz, I_yz, ...
                r_xp, r_xn, r_yp, r_yn, r_z, ...
                mass, f_min, f_max)
            %UAVMODEL Construct UAV model
            %   
            %obj = UAVMODEL(...
            %   I_xx, I_yy, I_zz, I_xy, I_xz, I_yz, ...
            %   r_xp, r_xn, r_yp, r_yn, r_z, ...
            %   m, f_min, f_max)
            %   Construct with params:
            %       I_xx = Inertia xx [kg*m^2]
            %       I_yy = IInertia yy [kg*m^2]
            %       I_zz = Inertia zz [kg*m^2]
            %       I_xy = Inertia xy [kg*m^2]
            %       I_xz = Inertia xz [kg*m^2]
            %       I_yz = Inertia yz [kg*m^2]
            %       r_xp = Moment arm x+ [m]
            %       r_xn = Moment arm x- [m]
            %       r_yp = Moment arm y+ [m]
            %       r_yn = Moment arm y- [m]
            %       r_z = Moment arm z [m]
            %       mass = Mass [kg]
            %       f_min = Min prop force [N]
            %       f_max = Max prop force [N]
            %   
            %obj = UAVMODEL()
            %   Construct with default params
            
            % Default model
            if nargin == 0
                I_xx = 1.15e-03;
                I_yy = 1.32e-03;
                I_zz = 2.24e-03;
                I_xy = 0;
                I_xz = 0;
                I_yz = 0;
                r_xp = 9.30e-02;
                r_xn = 9.30e-02;
                r_yp = 9.30e-02;
                r_yn = 9.30e-02;
                r_z = 5.50e-02;
                mass = 0.546;
                f_min = 0.00;
                f_max = 2.46;
            end
            
            % Copy parameters
            obj.I_xx = I_xx;
            obj.I_yy = I_yy;
            obj.I_zz = I_zz;
            obj.I_xy = I_xy;
            obj.I_xz = I_xz;
            obj.I_yz = I_yz;
            obj.r_xp = r_xp;
            obj.r_xn = r_xn;
            obj.r_yp = r_yp;
            obj.r_yn = r_yn;
            obj.r_z = r_z;
            obj.mass = mass;
            obj.f_min = f_min;
            obj.f_max = f_max;

            % Derived Matrices
            I_mat = [...
                [I_xx, I_xy, I_xz]; ...
                [I_xy, I_yy, I_yz]; ...
                [I_xz, I_yz, I_zz]];
            N_mat = [...
                [+r_xp, -r_xn, +r_xp, -r_xn]; ...
                [-r_yn, -r_yn, +r_yp, +r_yp]; ...
                [ +r_z,  -r_z,  -r_z,  +r_z]; ...
                [    1,     1,     1,     1]];
            M_sig = [...
                [I_mat, zeros(3, 1)];
                [zeros(1, 3), mass]];
            obj.I_mat = I_mat;
            obj.M_mat = N_mat \ M_sig;
            obj.M_alp = obj.M_mat(:,1:3);
            obj.M_acc = obj.M_mat(:,4:4);
        end
    end
end