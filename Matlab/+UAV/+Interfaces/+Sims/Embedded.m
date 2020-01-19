classdef Embedded < UAV.Interfaces.Sims.Sim
    %EMBEDDED Embedded simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte 
        msg_id_update = hex2dec('AA');  % Data msg ID
        baud_rate = 115200;             % Serial baud rate
        timeout = 0.5;                  % Serial timeout [s]
    end
    
    properties (Access = protected)
        remote;     % UAV remote controller [UAV.Interfaces.Remote]
        server;     % Serial interface [SerialServer]
        got_thr;    % Response flag [logical]
        thr_props;  % Prop throttles [0, 1]
    end
    
    methods (Access = public)
        function obj = Embedded(remote, sim_port)
            %obj = EMBEDDED(remote, sim_port)
            %   Construct UAV embedded simulator
            %   
            %   Inputs:
            %   - remote = UAV remote controller [UAV.Interfaces.Remote]
            %   - sim_port = USB serial port name [char]
            
            % Default args
            if nargin < 2, sim_port = 'COM17'; end
            if nargin < 1, remote = UAV.Interfaces.Remote(); end 
            
            % Copy properties
            obj@UAV.Interfaces.Sims.Sim(remote.model, remote.params);
            obj.remote = remote;
            
            % Set up simulation serial server
            serial_ = serial_com.make_serial(sim_port, obj.baud_rate);
            obj.server = serial_com.SerialServer(serial_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_update, 40, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 16, @obj.msg_rx_update);

            % Init fields
            obj.got_thr = false;
            obj.thr_props = zeros(4, 1);
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   
            %   Inputs:
            %   - cmd = UAV command [UAV.State.Cmd]
            %   
            %   Outputs:
            %   - state = UAV state [UAV.State.State]
            
            % Copy command
            obj.cmd = cmd;
            
            % Transmit command and state data
            obj.remote.update(cmd);
            obj.server.tx(obj.msg_id_update);
            
            % Get throttle commands
            obj.got_thr = false;
            timer = timing.Timer();
            while ~obj.got_thr
                obj.server.rx();
                if timer.elapsed(obj.timeout)
                    error('Serial timed out.');
                end
            end
            
            % Simulate dynamics
            state = obj.update_sim(obj.thr_props, cmd.enum);
        end
        
        function set_params(obj, params)
            %SET_PARAMS(obj, params)
            %   Set flight parameters
            %   Inputs:
            %       params = Flight params [UAV.Params]
            obj.params = params;
            obj.remote.set_params(params);
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
            %   - Angular position [float, [w; x; y; z]]
            %   - Angular velocity [float, [x; y; z], rad/s]
            %   - Local acceleration [float, [x; y; z], m/s^2]
            str = serial_com.Struct();
            str.set(obj.state.ang_pos.vector(), 'single');
            str.set(obj.state.ang_vel, 'single');
            acc_glo = obj.state.lin_acc;
            acc_loc = obj.state.ang_pos.inv().rotate(acc_glo);
            str.set(acc_loc, 'single');
            server.set_tx_data(str.get_buffer());
        end
        
        function msg_rx_update(obj, server)
            %MSG_RX_DATA(obj, server) Unpacks force RX message
            %   Data format:
            %   - Prop throttles [float, [++, +-, -+, --], [0, 1]]
            str = serial_com.Struct(server.get_rx_data());
            for i = 1:4
                obj.thr_props(i) = str.get('single');
            end
            obj.got_thr = true;
        end
    end
end