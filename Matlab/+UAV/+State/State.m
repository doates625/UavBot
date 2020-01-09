classdef State < handle
    %STATE Class for UAV state data
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        ang_pos;    % Orientation log [Quat]
        ang_vel;    % Local angular velocity [rad/s]
        lin_acc;    % Global linear acceleration [m/s^2]
        ang_z;      % Heading [rad]
        f_props;    % Propeller forces [N]
        enum;       % State machine enum [UAV.State.Enum]
    end
    
    methods (Access = public)
        function obj = State(ang_pos, ang_vel, lin_acc, f_props, enum)
            %obj = STATE(ang_pos, ang_vel, lin_acc, f_prop, state)
            %   Construct custom UAV state
            %   Inputs:
            %       ang_pos = Orientation [Quat]
            %       ang_vel = Local angular velocity [rad/s]
            %       lin_acc = Global linear acceleration [m/s^2]
            %       f_props = Propeller forces [N]
            %       enum = State machine enum [UAV.State.Enum]
            
            % Default args
            if nargin < 5, enum = UAV.State.Enum.Disabled; end
            if nargin < 4, f_props = zeros(4, 1); end
            if nargin < 3, lin_acc = zeros(3, 1); end
            if nargin < 2, ang_vel = zeros(3, 1); end
            if nargin < 1, ang_pos = Quat(); end
            
            % Copy states
            obj.ang_pos = ang_pos;
            obj.ang_vel = ang_vel;
            obj.lin_acc = lin_acc;
            obj.f_props = f_props;
            obj.enum = enum;
            
            % Compute heading
            x_hat = obj.ang_pos.rotate([1; 0; 0]);
            obj.ang_z = atan2(x_hat(2), x_hat(1));
        end
    end
end