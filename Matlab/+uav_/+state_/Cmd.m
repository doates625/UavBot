classdef Cmd < handle
    %CMD Class for UAV commands
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        ang_pos;    % Orientation cmd [quat.Quat]
        thr_lin;    % Linear throttle cmd [0, 1]
        enum;       % State machine enum cmd [uav.state.Enum]
    end
    
    methods
        function obj = Cmd(ang_pos, thr_lin, enum)
            %obj = CMD(ang_pos, thr_lin, enum)
            %   Construct UAV command
            %   
            %   Inputs:
            %   - ang_pos = Orientation cmd [quat.Quat]
            %   - thr_lin = Linear throttle cmd [0, 1]
            %   - enum = State machine enum cmd [uav.state.Enum]
            %   
            %   Arg ang_pos is formatted to be positive-w
            
            % Imports
            import('uav.state.Enum');
            import('quat.Quat');
            
            % Default args
            if nargin < 3, enum = Enum.Disabled; end
            if nargin < 2, thr_lin = 0.0; end
            if nargin < 1, ang_pos = Quat(); end
            
            % Copy components
            obj.ang_pos = pos_w(ang_pos);
            obj.thr_lin = thr_lin;
            obj.enum = enum;
        end
    end
end