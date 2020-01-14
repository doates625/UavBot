classdef Params < handle
    %PARAMS Class for UAV flight parameters
    %   Detailed explanation goes here
    
    properties (Access = protected)
        param_path = 'Params/';
    end
    
    properties (Access = public)
        file_name;  % Parameter file name [char]
        thr_min;    % Min linear throttle [0, 1]
        thr_max;    % Max linear throttle [0, 1]
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
    end
    
    methods (Access = public)
        function obj = Params(file_name)
            %obj = PARAMS(file_name) Create params with file name [char]
            %   If file_name exists, params will be loaded from the file
            %   Otherwise, file will be created with default params
            
            % File name
            if nargin < 1
                file_name = 'Params-Default';
            end
            obj.file_name = file_name;
            
            % Search for log file
            files = dir([obj.param_path, '*.mat']);
            found_file = false;
            for f = 1:length(files)
                if strcmp(files(f).name, [file_name, '.mat'])
                    found_file = true;
                    break
                end
            end
            
            % Assign params
            if found_file
                file = load(obj.full_path());
                obj = file.obj;
            else
                obj.thr_min = 0.2;
                obj.thr_max = 0.6;
                model = UAV.Model();
                poles = -5 + [-1e-5, 0, +1e-5];
                [obj.qx_kp, obj.qx_ki, obj.qx_kd] = ...
                    UAV.quat_gains(model.inr_xx, model.rad_x, model.f_max, poles);
                [obj.qy_kp, obj.qy_ki, obj.qy_kd] = ...
                    UAV.quat_gains(model.inr_yy, model.rad_y, model.f_max, poles);
                [obj.qz_kp, obj.qz_ki, obj.qz_kd] = ...
                    UAV.quat_gains(model.inr_zz, model.rad_z, model.f_max, poles);
                obj.qx_ff = 0;
                obj.qy_ff = 0;
                obj.qz_ff = 0;
                obj.save();
            end
        end
        
        function print_cpp(obj)
            %PRINT_CPP(obj) Prints C++ code for constants
            clc
            fprintf('UAV Control C++ Code:\n\n');
            
            % Control Constants
            fprintf('// Control Constants\n');
            fprintf('const float f_ctrl = %.1ff;\t\t// Control freq [Hz]\n', UAV.Model().f_ctrl);
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
        end
        
        function save(obj)
            %SAVE(obj) Saves params to mat file
            save(obj.full_path(), 'obj');
        end
    end
    
    methods (Access = protected)
        function path = full_path(obj)
            %path = FULL_PATH(obj) Full relative file path with '.mat' extension
            path = [obj.param_path, obj.file_name, '.mat'];
        end
    end
end