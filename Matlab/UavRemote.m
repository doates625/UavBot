classdef UavRemote < handle
    %UAVREMOTE Bluetooth remote control for UAV
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        name;   % Bluetooth device name [char]
        btooth; % Bluetooth interface [Bluetooth]
        server; % Serial interface [SerialServer]
    end
    
    methods (Access = public)
        function obj = UavRemote(name)
            %obj = UAVREMOTE(name) Construct remote interface
            %   name = Bluetooth device name [char]
            
            % TODO
        end
        function [q, w, acc, h] = send_cmds(obj, acc, h)
            %[q, w, acc, h] = SET_CMDS(obj, acc, h)
            %   Send commands and get state
            %   
            %   Inputs:
            %       acc = Global accel cmd [m/s^2]
            %       h = Heading cmd [rad]
            %   Outputs:
            %       q = Orientation [Quat]
            %       w = Local angular velocity [rad/s]
            %       acc = Local acceleration [m/s^2]
            %       h = Heading [rad]
            
            % TODO
        end
    end
end