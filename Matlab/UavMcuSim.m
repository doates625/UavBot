classdef UavMcuSim < UavSim
    %UAVMCUSIM Embedded simulator for UAV model
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte 
        msg_id_update = hex2dec('00');  % Data msg ID
        baud_rate = 115200;             % Serial baud rate
    end
    
    properties (Access = protected)
        remote;     % Remote controller [UavRemote]
        server;     % Serial interface [SerialServer]
        acc_loc;    % Local acceleration [m/s^2]
        forces;     % Propeller forces [N]
        got_rx;     % Data received flag [logical]
    end
    
    methods (Access = public)
        function obj = UavMcuSim(model, f_sim, remote, port)
            %obj = UAVMCUSIM(model, f_sim, remote)
            %   Construct UAV embedded simulator
            %   Inputs:
            %       model = UAV model [UAVModel]
            %       f_sim = Sim frequency [Hz]
            %       remote = Remote interface [UAVRemote]
            %       port = USB serial port name [char]
            
            % Copy properties
            obj = obj@UavSim(model, f_sim);
            obj.remote = remote;
            
            % Make and open serial port
            if nargin < 4, port = 'auto'; end
            serial_ = make_serial(port, obj.baud_rate);
            
            % Set up serial server
            obj.server = SerialServer(serial_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_update, 40, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 16, @obj.msg_rx_update);
            
            % Initialize vectors
            obj.acc_loc = zeros(3, 1);
            obj.forces = zeros(4, 1);
        end
        
        function [q, w, acc, tz, f, stat] = update(obj, acc_cmd, tz_cmd)
            %[q, w, acc, tz, f, stat] = UPDATE(obj, acc_cmd, tz_cmd)
            %   Run simulation iteration and get states
            %   
            %   Inputs:
            %       acc_cmd = Global accel cmd [m/s^2]
            %       tz_cmd = Heading cmd [rad]
            %   Outputs:
            %       q = Orientation [Quat]
            %       w = Local angular velocity [rad/s]
            %       acc = Global accel [m/s^2]
            %       tz = Heading [rad]
            %       f = Propeller forces [N]
            %       stat = Status [0 = OK, 1 = failed]
            
            % Transmit commands and state data
            obj.remote.update(acc_cmd, tz_cmd);
            obj.server.tx(obj.msg_id_update);
            
            % Get force data
            obj.got_rx = false;
            while ~obj.got_rx
                obj.server.rx();
            end
            f = obj.forces;
            
            % Simulate system
            [q, w, acc] = obj.update_sim(f);
            [tz, stat] = obj.proc_quat();
            
            % Compute local accel
            obj.acc_loc = obj.q.inv().rotate(acc);
        end
        
        function disconnect(obj)
            %DISCONNECT(obj) Disconnects from serial port
            fclose(obj.server.get_serial());
        end
        
        function delete(obj)
            %DELETE(obj) Disconnects from serial then destructs
            obj.disconnect();
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
            str.set(obj.q.vector(), 'single');
            str.set(obj.w, 'single');
            str.set(obj.acc_loc, 'single');
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
                obj.forces(i) = str.get('single');
            end
            obj.got_rx = true;
        end
    end
end