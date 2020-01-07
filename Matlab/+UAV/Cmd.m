classdef Cmd < handle
    %CMD Class for UAV commands
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = public, Constant)
        valid_states = {...
            'Enabled'; ...
            'Disabled'; ...
            'Failed'};
    end
    
    properties (SetAccess = protected)
        acc;    % Global acceleration cmd [m/s^2]
        tz;     % Heading cmd [rad]
        state;  % State cmd [char] ['Enabled', 'Disabled', 'Failed']
    end
    
    methods
        function obj = Cmd(acc, tz, state)
            %obj = CMD(acc, tz, state) Construct UAV command
            %   Inputs:
            %       acc = Global acceleration cmd [m/s^2]
            %       tz = Heading cmd [rad]
            %       state = State cmd [char]
            %   Arg state must be one of Cmd.valid_states
            obj.acc = acc;
            obj.tz = tz;
            if any(strcmp(state, obj.valid_states))
                obj.state = state;
            else
                error('Invalid state: %s', state)
            end
        end
    end
end