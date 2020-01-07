classdef State < handle
    %STATE Class for UAV state data
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        ang_pos;    % Orientation [Quat]
        ang_vel;    % Local angular velocity [rad/s]
        lin_acc;    % Global linear acceleration [m/s^2]
        f_prop;     % Prop forces [N] [++; +-; -+; --]
        state;      % State string [char]
    end
    
    methods (Access = public)
        function obj = State(ang_pos, ang_vel, lin_acc, f_prop, state)
            %obj = STATE(ang_pos, ang_vel, lin_acc, f_prop, state)
            %   Construct custom UAV state
            %   Inputs:
            %       ang_pos = Orientation [Quat]
            %       ang_vel = Local angular velocity [rad/s]
            %       lin_acc = Global linear acceleration [m/s^2]
            %       f_prop = Prop forces [N] [++; +-; -+; --]
            %       state = State string [char]
            
            % Default args
            if nargin < 5, state = 'Disabled'; end
            if nargin < 4, f_prop = zeros(4, 1); end
            if nargin < 3, lin_acc = zeros(3, 1); end
            if nargin < 2, ang_vel = zeros(3, 1); end
            if nargin < 1, ang_pos = Quat(); end
            
            % Copy states
            obj.ang_pos = ang_pos;
            obj.ang_vel = ang_vel;
            obj.lin_acc = lin_acc;
            obj.f_prop = f_prop;
            obj.state = state;
        end
        
        function ang_z = get_ang_z(obj)
            %ang_z = GET_ANG_Z(obj) Get UAV heading [rad] [-pi, +pi]
            x_hat = obj.ang_pos.rotate([1; 0; 0]);
            ang_z = atan2(x_hat(2), x_hat(1));
        end
    end
end