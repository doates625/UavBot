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
            %   
            %   If file_name exists, params will be loaded from the file
            %   Otherwise, file will be created with default params
            
            % Imports
            import('UAV.Model');
            import('UAV.quat_gains');
            
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
                model = Model();
                poles = -5 + [-1e-5, 0, +1e-5];
                [obj.qx_kp, obj.qx_ki, obj.qx_kd] = ...
                    quat_gains(model.inr_xx, model.rad_x, model.f_max, poles);
                [obj.qy_kp, obj.qy_ki, obj.qy_kd] = ...
                    quat_gains(model.inr_yy, model.rad_y, model.f_max, poles);
                [obj.qz_kp, obj.qz_ki, obj.qz_kd] = ...
                    quat_gains(model.inr_zz, model.rad_z, model.f_max, poles);
                obj.qx_ff = 0;
                obj.qy_ff = 0;
                obj.qz_ff = 0;
                obj.save();
            end
        end
        
        function save(obj)
            %SAVE(obj) Saves params to mat file
            save(obj.full_path(), 'obj');
        end
    end
    
    methods (Access = protected)
        function path = full_path(obj)
            %path = FULL_PATH(obj)
            %   Full relative file path with '.mat' extension
            path = [obj.param_path, obj.file_name, '.mat'];
        end
    end
end