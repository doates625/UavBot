classdef Model < handle
    %MODEL UAV physical and control constants
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (GetAccess = public, Constant)
        gravity_sca = 9.807;
        gravity_vec = [0; 0; UAV.Model.gravity_sca];
        N_mat = [...
            [+1, -1, +1, +1]; ...
            [-1, -1, -1, +1]; ...
            [+1, +1, -1, +1]; ...
            [-1, +1, +1, +1]];
        N_ang = UAV.Model.N_mat(:, 1:3);
        N_lin = UAV.Model.N_mat(:, 4:4);
        N_inv = inv(UAV.Model.N_mat);
        N_inv_ang = UAV.Model.N_inv(1:3, :);
        N_inv_lin = UAV.Model.N_inv(4:4, :);
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
        f_max;      % Max single prop force [N]
        f_ctrl;     % Control frequency [Hz]
        t_ctrl;     % Control period [s]
    end
    
    methods (Access = public)
        function obj = Model(...
                inr_xx, inr_yy, inr_zz, mass, ...
                rad_x, rad_y, rad_z, ...
                f_max, f_ctrl)
            %MODEL Construct UAV model
            %   obj = MODEL(...
            %       inr_xx, inr_yy, inr_zz, mass, ...
            %       rad_x, rad_y, rad_z, ...
            %       f_max, f_ctrl)
            %       Construct custom UAV model
            %       Inputs:
            %           inr_xx = Inertia moment x [kg*m^2]
            %           inr_yy = Inertia moment y [kg*m^2]
            %           inr_zz = Inertia moment z [kg*m^2]
            %           mass = Mass [kg]
            %           rad_x = Prop moment arm x [m]
            %           rad_y = Prop moment arm y [m]
            %           rad_z = Prop moment arm z [m]
            %           f_max = Max single prop force [N]
            %           f_ctrl = Control frequency [Hz]
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
                f_max = 2.46;
                f_ctrl = 50.0;
            elseif nargin ~= 9
                error('Invalid nargin.')
            end
            
            % Copy constants
            obj.inr_xx = inr_xx;
            obj.inr_yy = inr_yy;
            obj.inr_zz = inr_zz;
            obj.mass = mass;
            obj.rad_x = rad_x;
            obj.rad_y = rad_y;
            obj.rad_z = rad_z;
            obj.f_max = f_max;
            obj.f_ctrl = f_ctrl;
            obj.t_ctrl = 1 / f_ctrl;
        end
        
        function print_cpp(obj)
            %PRINT_CPP(obj) Prints C++ code for constants
            clc
            fprintf('UAV Model C++ Code:\n\n');
            
            % Init angular N-matrix
            fprintf('// Init angular N-matrix\n');
            print_mat_cpp('N_ang', obj.N_ang);
            fprintf('\n');
            
            % Init linear N-matrix
            fprintf('// Init linear N-matrix\n');
            print_mat_cpp('N_lin', obj.N_lin);
            fprintf('\n');
        end
    end
end

