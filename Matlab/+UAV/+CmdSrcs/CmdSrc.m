classdef (Abstract) CmdSrc < handle
    %CMDSRC Superclass for UAV command sources
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model;  % UAV model [UAV.Model]
        params; % Flight params [UAV.Params]
    end
    
    methods (Access = public)
        function obj = CmdSrc(model, params)
            %obj = CMDSRC(model, params)
            %   Construct command source
            %   
            %   Inputs:
            %   - model = UAV model [UAV.Model]
            %   - params = Flight params [UAV.Params]
            obj.model = model;
            obj.params = params;
        end
    end
    
    methods (Access = public, Abstract)
        [cmd, time] = get_cmd(obj)
        %[cmd, time] = GET_CMD(obj) Get commands and time
        %   
        %   Outputs:
        %   - cmd = UAV command [UAV.State.Cmd]
        %   - time = Time [s]
        
        stop = get_stop(obj)
        %stop = GET_STOP(obj) Get stop flag [logical]
    end
    
    methods (Access = protected, Static)
        function quat_ = eul_to_quat(ang_z, ang_y, ang_x)
            %quat_ = EUL_TO_QUAT(ang_z, ang_y, ang_x)
            %   Convert euler angle(s) to quaternion(s)
            %   
            %   Inputs:
            %   - ang_z = Array of yaw [rad]
            %   - ang_y = Array of pitch [rad]
            %   - ang_x = Array of roll [rad]
            %   
            %   Outputs:
            %   - quat_ = Array of Quat
            import('quat.Quat');
            N = length(ang_z);
            quat_(1, N) = Quat();
            for i = 1:length(ang_z)
                qz = Quat([0; 0; 1], ang_z(i));
                qy = Quat([0; 1; 0], ang_y(i));
                qx = Quat([1; 0; 0], ang_x(i));
                quat_(i) = pos_w(qz * qy * qx);
            end
        end
    end
end