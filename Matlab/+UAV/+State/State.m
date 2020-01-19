classdef State < handle
    %STATE Class for UAV state data
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        ang_pos;    % Orientation [Quat]
        ang_vel;    % Local angular velocity [rad/s]
        lin_acc;    % Global linear acceleration [m/s^2]
        thr_props;  % Prop throttles [0, 1]
        enum;       % State machine enum [UAV.State.Enum]
    end
    
    methods (Access = public)
        function obj = State(ang_pos, ang_vel, lin_acc, thr_props, enum)
            %obj = STATE(ang_pos, ang_vel, lin_acc, thr_props, state)
            %   Construct custom UAV state
            %   Inputs:
            %       ang_pos = Orientation [Quat]
            %       ang_vel = Local angular velocity [rad/s]
            %       lin_acc = Global linear acceleration [m/s^2]
            %       thr_prop = Prop throttles [0, 1]
            %       enum = State machine enum [UAV.State.Enum]
            
            % Default args
            if nargin < 5, enum = UAV.State.Enum.Disabled; end
            if nargin < 4, thr_props = zeros(4, 1); end
            if nargin < 3, lin_acc = zeros(3, 1); end
            if nargin < 2, ang_vel = zeros(3, 1); end
            if nargin < 1, ang_pos = quat.Quat(); end
            
            % Copy states
            obj.ang_pos = pos_w(ang_pos);
            obj.ang_vel = ang_vel;
            obj.lin_acc = lin_acc;
            obj.thr_props = thr_props;
            obj.enum = enum;
        end
    end
end