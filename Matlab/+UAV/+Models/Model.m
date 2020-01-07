classdef Model < handle
    %MODEL UAV physical and control model parameters
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        phys;   % Physical model [UAV.Models.Phys]
        ctrl;   % Control model [UAV.Models.Ctrl]
    end
    
    methods (Access = public)
        function obj = Model(phys, ctrl)
            %obj = MODEL(phys, ctrl) Construct UAV model
            %   phys = Physical model [UAV.Models.Phys]
            %   ctrl = Control model [UAV.Models.Ctrl]
            
            % Default args
            import('UAV.default_arg');
            if nargin < 2, ctrl = default_arg('model_ctrl'); end
            if nargin < 1, phys = default_arg('model_phys'); end
            
            % Pointers
            obj.phys = phys;
            obj.ctrl = ctrl;
        end
    end
end