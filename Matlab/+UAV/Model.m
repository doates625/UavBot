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
    %       acc = Acceleration [m/s^2]
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
        
        % Control Constants
        f_ctrl;     % Control frequency [Hz]
        t_ctrl;     % Control period [s]
        thr_min;    % Min linear throttle
        thr_max;    % Max linear throttle
        qx_kp;      % Quat-x P-gain [thr/rad]
        qx_ki;      % Quat-x I-gain [thr/(rad*s)]
        qx_kd;      % Quat-x D-gain [thr/(rad/s)]
        qx_ff;      % Quat-x feed-forward [thr]
        qy_kp;      % Quat-y P-gain [thr/rad]
        qy_ki;      % Quat-y I-gain [thr/(rad*s)]
        qy_kd;      % Quat-y D-gain [thr/(rad/s)]
        qy_ff;      % Quat-y feed-forward [thr]
        qz_kp;      % Quat-z P-gain [thr/rad]
        qz_ki;      % Quat-z I-gain [thr/(rad*s)]
        qz_kd;      % Quat-z D-gain [thr/(rad/s)]
        qz_ff;      % Quat-z feed-forward [thr]
        
        % Derived Constants
        A_ang;  % Angular acc matrix [rad/s^2]
        A_lin;  % Linear acc matrix [m/s^2]
    end
    
    methods (Access = public)
        function obj = Model(...
                inr_xx, inr_yy, inr_zz, mass, ...
                rad_x, rad_y, rad_z, f_max, ...
            	f_ctrl, thr_min, thr_max, ...
            	qx_kp, qx_ki, qx_kd, qx_ff, ...
            	qy_kp, qy_ki, qy_kd, qy_ff, ...
            	qz_kp, qz_ki, qz_kd, qz_ff)
            %MODEL Construct UAV model
            %   obj = MODEL(...
            %       inr_xx, inr_yy, inr_zz, mass, ...
            %       rad_x, rad_y, rad_z, f_max, ...
            %       f_ctrl, thr_min, thr_max, ...
            %       qx_kp, qx_ki, qx_kd, qx_ff, ...
            %       qy_kp, qy_ki, qy_kd, qy_ff, ...
            %       qz_kp, qz_ki, qz_kd, qz_ff)
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
            %           thr_min = Min linear throttle
            %           thr_max = Max linear throttle
            %           qx_kp = Quat-x P-gain [thr/rad]
            %           qx_ki = Quat-x I-gain [thr/(rad*s)]
            %           qx_kd = Quat-x D-gain [thr/(rad/s)]
            %           qx_ff = Quat-x feed-forward [thr]
            %           qy_kp = Quat-y P-gain [thr/rad]
            %           qy_ki = Quat-y I-gain [thr/(rad*s)]
            %           qy_kd = Quat-y D-gain [thr/(rad/s)]
            %           qy_ff = Quat-y feed-forward [thr]
            %           qz_kp = Quat-z P-gain [thr/rad]
            %           qz_ki = Quat-z I-gain [thr/(rad*s)]
            %           qz_kd = Quat-z D-gain [thr/(rad/s)]
            %           qz_ff = Quat-z feed-forward [thr]
            %   obj = MODEL() Construct default model
            
            % Default args
            if nargin == 0
                % Physical constants
                inr_xx = 1.15e-03;
                inr_yy = 1.32e-03;
                inr_zz = 2.24e-03;
                mass = 0.546;
                rad_x = 9.30e-02;
                rad_y = 9.30e-02;
                rad_z = 5.50e-02;
                f_max = 2.46;
                
                % Control constants
                f_ctrl = 50.0;
                thr_min = 0.5;
                thr_max = 0.6;
                poles = -5 + [-1e-5, 0, +1e-5];
                [qx_kp, qx_ki, qx_kd] = obj.make_pid(f_max, rad_x, inr_xx, poles);
                [qy_kp, qy_ki, qy_kd] = obj.make_pid(f_max, rad_y, inr_yy, poles);
                [qz_kp, qz_ki, qz_kd] = obj.make_pid(f_max, rad_z, inr_zz, poles);
                qx_ff = 0.0;
                qy_ff = 0.0;
                qz_ff = 0.0;
            elseif nargin ~= 24
                error('Invalid nargin.')
            end
            
            % Physical constants
            obj.inr_xx = inr_xx;
            obj.inr_yy = inr_yy;
            obj.inr_zz = inr_zz;
            obj.mass = mass;
            obj.rad_x = rad_x;
            obj.rad_y = rad_y;
            obj.rad_z = rad_z;
            obj.f_max = f_max;
        
            % Control constants
            obj.f_ctrl = f_ctrl;
            obj.t_ctrl = 1 / f_ctrl;
            obj.thr_min = thr_min;
            obj.thr_max = thr_max;
            obj.qx_kp = qx_kp;
            obj.qx_ki = qx_ki;
            obj.qx_kd = qx_kd;
            obj.qx_ff = qx_ff;
            obj.qy_kp = qy_kp;
            obj.qy_ki = qy_ki;
            obj.qy_kd = qy_kd;
            obj.qy_ff = qy_ff;
            obj.qz_kp = qz_kp;
            obj.qz_ki = qz_ki;
            obj.qz_kd = qz_kd;
            obj.qz_ff = qz_ff;
            
            % Acceleration matrices
            I_mat = diag([inr_xx, inr_yy, inr_zz]);
            D_mat = [...
                [+rad_x, -rad_x, +rad_x, -rad_x]; ...
                [-rad_y, -rad_y, +rad_y, +rad_y]; ...
                [+rad_z, -rad_z, -rad_z, +rad_z]];
            obj.A_ang = f_max * inv(I_mat) * D_mat;
            obj.A_lin = zeros(3, 4);
            obj.A_lin(3, :) = f_max / mass;
        end
        
        function print_cpp(obj)
            %PRINT_CPP(obj) Prints C++ code for constants
            clc
            fprintf('UAV Model C++ Code:\n\n')
            
            % Control Constants
            fprintf('// Control Constants\n');
            fprintf('const float f_ctrl = %.1ff;\t\t// Control freq [Hz]\n', obj.f_ctrl);
            fprintf('const float thr_min = %.2ff;\t// Min linear throttle\n', obj.thr_min);
            fprintf('const float thr_max = %.2ff;\t// Max linear throttle\n', obj.thr_max);
            fprintf('\n');
            
            % Quat X-axis Gains
            fprintf('// Quat X-axis Gains\n');
            fprintf('const float qx_kp = %+.3ff;\t// P-gain\n', obj.qx_kp);
            fprintf('const float qx_ki = %+.3ff;\t// I-gain\n', obj.qx_ki);
            fprintf('const float qx_kd = %+.3ff;\t// D-gain\n', obj.qx_kd);
            fprintf('const float qx_ff = %+.3ff;\t// Feed-forward\n', obj.qx_ff);
            fprintf('\n');
            
            % Quat Y-axis Gains
            fprintf('// Quat Y-axis Gains\n');
            fprintf('const float qy_kp = %+.3ff;\t// P-gain\n', obj.qy_kp);
            fprintf('const float qy_ki = %+.3ff;\t// I-gain\n', obj.qy_ki);
            fprintf('const float qy_kd = %+.3ff;\t// D-gain\n', obj.qy_kd);
            fprintf('const float qy_ff = %+.3ff;\t// Feed-forward\n', obj.qy_ff);
            fprintf('\n');
            
            % Quat Z-axis Gains
            fprintf('// Quat Z-axis Gains\n');
            fprintf('const float qz_kp = %+.3ff;\t// P-gain\n', obj.qz_kp);
            fprintf('const float qz_ki = %+.3ff;\t// I-gain\n', obj.qz_ki);
            fprintf('const float qz_kd = %+.3ff;\t// D-gain\n', obj.qz_kd);
            fprintf('const float qz_ff = %+.3ff;\t// Feed-forward\n', obj.qz_ff);
            fprintf('\n');
            
            % Init angular N-matrix
            fprintf('// Init angular N-matrix\n')
            print_mat_cpp('N_ang', obj.N_ang);
            fprintf('\n')
            
            % Init linear N-matrix
            fprintf('// Init linear N-matrix\n')
            print_mat_cpp('N_lin', obj.N_lin);
            fprintf('\n')
        end
    end
    
    methods (Access = protected, Static)
        function [kp, ki, kd] = make_pid(f_max, rad, inr, poles)
            %[kp, ki, kd] = MAKE_PID(f_max, rad, inr, poles)
            %   Make quaternion PID controller
            %   Inputs:
            %       f_max = Max prop force [N]
            %       rad = Moment arm radius [m]
            %       inr = Moment of inertia [kg*m^2]
            %       poles = Three poles [s1, s2, s3]
            %   Outputs:
            %       kp = P-gain [thr/rad]
            %       ki = I-gain [thr/(rad*s)]
            %       kd = D-gain [thr/(rad/s)]
            A = zeros(3, 3);
            A(1, 2) = 1;
            A(2, 3) = 1;
            B = zeros(3, 1);
            H = 2*f_max*rad/inr;
            B(3) = H;
            K = place(A, B, poles);
            ki = K(1);
            kp = K(2);
            kd = K(3);
        end
    end
end

