classdef Embedded < UAV.Interfaces.Sims.Sim
    %EMBEDDED Embedded simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte 
        msg_id_update = hex2dec('00');  % Data msg ID
        baud_rate = 115200;             % Serial baud rate
    end
    
    properties (Access = protected)
        remote;     % UAV remote controller [UAV.Interfaces.Remote]
        server;     % Serial interface [SerialServer]
        got_resp;   % Response flag [logical]
        f_prop;     % Prop forces [N]
    end
    
    methods (Access = public)
        function obj = Embedded(phys_model, f_sim, remote, sim_port)
            %obj = EMBEDDED(phys_model, f_sim, remote, sim_port)
            %   Construct UAV embedded simulator
            %   Inputs:
            %       phys_model = UAV physical model [UAV.Models.Phys]
            %       f_sim = Simulation frequency [Hz]
            %       remote = UAV remote controller [UAV.Interfaces.Remote]
            %       sim_port = USB serial port name [char]
            
            % Default args
            import('UAV.default_arg');
            if nargin < 4, sim_port = default_arg('sim_port'); end
            if nargin < 3, remote = default_arg('remote'); end
            if nargin < 2, f_sim = default_arg('f_sim'); end
            if nargin < 1, phys_model = default_arg('phys_model'); end
            
            % Copy properties
            obj = obj@UAV.Interfaces.Sims.Sim(phys_model, f_sim);
            obj.remote = remote;
            
            % Set up simulation serial server
            serial_ = make_serial(sim_port, obj.baud_rate);
            obj.server = SerialServer(serial_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_update, 40, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 16, @obj.msg_rx_update);

            % Init fields
            obj.f_prop = zeros(4, 1);
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   Inputs:
            %       cmd = UAV command [UAV.Cmd]
            %   Outputs:
            %       state = UAV state [UAV.State]
            
            % Transmit commands and state data
            obj.remote.update(cmd);
            obj.server.tx(obj.msg_id_update);
            
            % Get force commands
            obj.got_resp = false;
            while ~obj.got_resp
                obj.server.rx();
            end
            
            % Simulate dynamics
            state = obj.update_sim(obj.f_prop, cmd.state);
        end
        
        function delete(obj)
            %DELETE(obj) Disconnects from serial port
            fclose(obj.server.get_serial());
        end
    end
    
    methods (Access = protected)
        function msg_tx_update(obj, server)
            %MSG_TX_DATA(obj, server) Packs IMU reading TX message
            %   Data format:
            %   [00-03] Quat-w [float]
            %   [04-07] Quat-x [float]
            %   [08-11] Quat-y [float]
            %   [12-15] Quat-z [float]
            %   [16-19] Omega-x [float, rad/s]
            %   [20-23] Omega-y [float, rad/s]
            %   [24-27] Omega-z [float, rad/s]
            %   [28-31] Local accel-x [float, m/s^2]
            %   [32-35] Local accel-y [float, m/s^2]
            %   [36-39] Local accel-z [float, m/s^2]
            str = Struct();
            str.set(obj.state.ang_pos.vector(), 'single');
            str.set(obj.state.ang_vel, 'single');
            acc_loc = obj.state.ang_pos.inv().rotate(obj.state.lin_acc);
            str.set(acc_loc, 'single');
            server.set_tx_data(str.get_buffer());
        end
        
        function msg_rx_update(obj, server)
            %MSG_RX_DATA(obj, server) Unpacks force RX message
            %   Data format:
            %   [00-03] Force++ [float, N]
            %   [04-07] Force+- [float, N]
            %   [08-11] Force-+ [float, N]
            %   [12-15] Force-- [float, N]
            str = Struct(server.get_rx_data());
            for i = 1:4
                obj.f_prop(i) = str.get('single');
            end
            obj.got_resp = true;
        end
    end
end

