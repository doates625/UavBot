classdef UavEmbeddedSim < UavSim
    %UAVEMBEDDEDSIM Embedded simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function obj = UavEmbeddedSim(model, f_sim, remote)
            %obj = UAVEMBEDDEDSIM(model, f_sim, remote)
            %   Construct UAV embedded simulator
            %   
            %   Inputs:
            %       model = UAV model [UAVModel]
            %       f_sim = Sim frequency [Hz]
            %       remote = Remote interface [UAVRemote]
            
            % TODO
        end
        
        function [q, w, acc, tz, f, stat] = update(obj, acc_cmd, tz_cmd)
            %[q, w, acc, tz, f, stat] = UPDATE(obj, acc_cmd, tz_cmd)
            %   Run simulation iteration and get states
            %   
            %   Inputs:
            %       acc_cmd = Global accel cmd [m/s^2]
            %       tz_cmd = Heading cmd [rad]
            %   Outputs:
            %       q = Orientation [Quat]
            %       w = Local angular velocity [rad/s]
            %       acc = Global accel [m/s^2]
            %       tz = Heading [rad]
            %       f = Propeller forces [N]
            %       stat = Status [0 = OK, 1 = failed]
            
            % TODO
        end
    end
end