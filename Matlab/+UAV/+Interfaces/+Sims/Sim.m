classdef (Abstract) Sim < UAV.Interfaces.Interface
    %SIM Superclass for simulated UAVs
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = public, Constant)
        f_sim = 50.0;   % Simulation frequency [Hz]
        t_sim = 0.02;   % Simulation period [s]
    end
    
    properties (SetAccess = protected)
        phys_model; % UAV physical model [UAV.Models.Phys]
    end
    
    methods (Access = public)
        function obj = Sim(phys_model)
            %obj = SIM(phys_model) Construct UAV simulator
            %   Inputs:
            %       phys_model = UAV physical model [UAV.Models.Phys]
            
            % Superconstructor
            obj = obj@UAV.Interfaces.Interface();
            
            % Physical model
            if nargin < 1
                phys_model = UAV.Models.Phys();
            end
            obj.phys_model = phys_model;
        end
    end
    
    methods (Access = protected)
        function state = update_sim(obj, f_prop, state_cmd)
            %state = UPDATE_SIM(obj, f_prop, state_cmd) Run one simulation iteration
            %   Inputs:
            %       f = Prop force vector [N]
            %       state_cmd = State cmd [char]
            %   Outputs:
            %       state = UAV state [UAV.State]
            
            % State machine
            ang_pos = obj.state.ang_pos;
            z_hat = ang_pos.rotate([0; 0; 1]);
            if z_hat(3) > 0
                state = state_cmd;
            else
                state = 'Failed';
            end
            
            % Prop force override
            if ~strcmp(state, 'Enabled')
                f_prop = zeros(4, 1);
            end
            
            % Angular dynamics
            ang_acc = obj.phys_model.M_bar_ang * f_prop;
            ang_vel = obj.state.ang_vel;
            norm_ang_vel = norm(ang_vel);
            if norm_ang_vel > 0
                theta = norm_ang_vel * obj.t_sim;
                ang_pos = ang_pos * Quat(ang_vel, theta);
            end
            ang_vel = ang_vel + ang_acc * obj.t_sim;
            
            % Linear dynamics
            lin_acc = obj.phys_model.M_bar_lin * f_prop;
            lin_acc = [0; 0; lin_acc];
            lin_acc = ang_pos.rotate(lin_acc) - obj.phys_model.g_vec;
            
            % Set state
            obj.state = UAV.State(ang_pos, ang_vel, lin_acc, f_prop, state);
            state = obj.state;
        end
    end
end

