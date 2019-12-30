classdef UavRemote < UavInterface
    %UAVREMOTE Bluetooth remote control for UAV
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte
        msg_id_start = hex2dec('00');   % Start msg ID
        msg_id_update = hex2dec('01');  % Update msg ID
    end
    
    properties (Access = protected)
        server;     % Serial interface [SerialServer]
        acc_cmd;    % Global accel cmd [rad/s]
        tz_cmd;     % Heading cmd [rad]
        acc;        % Global accel [m/s^2]
        f;          % Propeller forces [N]
        got_rx;     % Respone flag [logical]
    end
    
    methods (Access = public)
        function obj = UavRemote(name)
            %obj = UAVREMOTE(name) Construct Bluetooth remote control
            %   name = Bluetooth device name [char]
            
            % Make and open Bluetooth
            bluetooth_ = make_bluetooth(name);
            
            % Set up serial server
            obj.server = SerialServer(bluetooth_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_start, 0, @obj.msg_tx_start);
            obj.server.add_tx(obj.msg_id_update, 16, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 56, @obj.msg_rx_update);
            
            % Tell UAV to start
            obj.server.tx(obj.msg_id_start);
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
            
            % Transmit commands
            obj.acc_cmd = acc_cmd;
            obj.tz_cmd = tz_cmd;
            obj.server.tx(obj.msg_id_update);
            
            % Wait for response
            obj.got_rx = false;
            while ~obj.got_rx
                obj.server.rx();
            end
            
            % Process response
            q = obj.q;
            w = obj.w;
            acc = obj.acc;
            f = obj.f;
            [tz, stat] = obj.proc_quat();
        end
        
        function disconnect(obj)
            %DISCONNECT(obj) Disconnects from Bluetooth
            fclose(obj.server.get_serial());
        end
        
        function delete(obj)
            %DELETE(obj) Disconnects from Bluetooth then destructs
            obj.disconnect();
        end
    end
    
    methods (Access = protected)
        function msg_tx_start(~, server)
            %MSG_TX_START(obj, server) Packs Start TX message
            server.set_tx_data([]);
        end
        
        function msg_tx_update(obj, server)
            %MSG_TX_UPDATE(obj, server) Packs Update TX message
            %   Data format:
            %   [00-03] Global accel x [float, m/s^2]
            %   [04-07] Global accel y [float, m/s^2]
            %   [08-11] Global accel z [float, m/s^2]
            %   [12-15] Heading cmd [float, rad]
            str = Struct();
            str.set(obj.acc_cmd, 'single');
            str.set(obj.tz_cmd, 'single');
            server.set_tx_data(str.get_buffer());
        end
        
        function msg_rx_update(obj, server)
            %MSG_RX_UPDAT(obj, server) Unpacks Update RX message
            %   Data format:
            %   [00-03] Quat-w [float]
            %   [04-07] Quat-x [float]
            %   [08-11] Quat-y [float]
            %   [12-15] Quat-z [float]
            %   [16-19] Omega-x [float, rad/s]
            %   [20-23] Omega-y [float, rad/s]
            %   [24-27] Omega-z [float, rad/s]
            %   [28-31] Accel-x [float, m/s^2]
            %   [32-35] Accel-y [float, m/s^2]
            %   [36-39] Accel-z [float, m/s^2]
            %   [40-43] Force++ [float, N]
            %   [44-47] Force+- [float, N]
            %   [48-51] Force-+ [float, N]
            %   [52-55] Force-- [float, N]
            str = Struct(server.get_rx_data());
            arr = zeros(14, 1);
            for i = 1:length(arr)
                arr(i) = str.get('single');
            end
            obj.q = Quat(arr(1:4));
            obj.w = arr(5:7);
            obj.acc = obj.q.rotate(arr(8:10));
            obj.f = arr(11:14);
            obj.got_rx = true;
        end
    end
end