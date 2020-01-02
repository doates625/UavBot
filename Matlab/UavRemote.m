classdef UavRemote < UavInterface
    %UAVREMOTE Bluetooth remote control for UAV
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte
        msg_id_enable = hex2dec('00');  % Enable msg ID
        msg_id_update = hex2dec('01');  % Update msg ID
    end
    
    properties (Access = protected)
        server;     % Serial interface [SerialServer]
        accel;      % Global accel [m/s^2]
        forces;     % Propeller forces [N]
        got_rx;     % Respone flag [logical]
    end
    
    properties (SetAccess = protected)
        enable_cmd; % Flight enable cmd [logical]
        acc_cmd;    % Global accel cmd [rad/s]
        tz_cmd;     % Heading cmd [rad]
        state;      % State string [char]
    end
    
    methods (Access = public)
        function obj = UavRemote(name)
            %obj = UAVREMOTE(name) Construct Bluetooth remote control
            %   name = Bluetooth device name [char]
            
            % Make and open Bluetooth
            bluetooth_ = make_bluetooth(name);
            
            % Set up serial server
            obj.server = SerialServer(bluetooth_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_enable, 0, @obj.msg_tx_enable);
            obj.server.add_tx(obj.msg_id_update, 16, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 57, @obj.msg_rx_update);
            
            % Init properties
            obj.enable_cmd = false;
            obj.acc_cmd = zeros(3, 1);
            obj.tz_cmd = 0.0;
            obj.accel = zeros(3, 1);
            obj.forces = zeros(4, 1);
            obj.got_rx = false;
        end
        
        function set_enabled(obj, enabled)
            %SET_ENABLED(obj, enabled) Sets flight enable [logical]
            obj.enable_cmd = enabled;
            obj.server.tx(obj.msg_id_enable);
        end
        
        function [q, w, acc, tz, f, stat] = update(obj, acc_cmd, tz_cmd)
            %[q, w, acc, tz, f, stat] = UPDATE(obj, acc_cmd, tz_cmd)
            %   Run simulation iteration and get states
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
            acc = obj.accel;
            f = obj.forces;
            [tz, stat] = obj.proc_quat();
        end
        
        function state = get_state(obj)
            %state = GET_STATE(obj) Returns state string
            state = obj.state;
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
        function msg_tx_enable(obj, server)
            %MSG_TX_START(obj, server) Packs Enable TX message
            server.set_tx_data(obj.enable_cmd);
        end
        
        function msg_tx_update(obj, server)
            %MSG_TX_UPDATE(obj, server) Packs Update TX message
            %   Data format:
            %   [00-03] Global accel x [single, m/s^2]
            %   [04-07] Global accel y [single, m/s^2]
            %   [08-11] Global accel z [single, m/s^2]
            %   [12-15] Heading cmd [single, rad]
            str = Struct();
            str.set(obj.acc_cmd, 'single');
            str.set(obj.tz_cmd, 'single');
            server.set_tx_data(str.get_buffer());
        end
        
        function msg_rx_update(obj, server)
            %MSG_RX_UPDAT(obj, server) Unpacks Update RX message
            %   Data format:
            %   [00-03] Quat-w [single]
            %   [04-07] Quat-x [single]
            %   [08-11] Quat-y [single]
            %   [12-15] Quat-z [single]
            %   [16-19] Omega-x [single, rad/s]
            %   [20-23] Omega-y [single, rad/s]
            %   [24-27] Omega-z [single, rad/s]
            %   [28-31] Accel-x [single, m/s^2]
            %   [32-35] Accel-y [single, m/s^2]
            %   [36-39] Accel-z [single, m/s^2]
            %   [40-43] Force++ [single, N]
            %   [44-47] Force+- [single, N]
            %   [48-51] Force-+ [single, N]
            %   [52-55] Force-- [single, N]
            %   [56-56] State [uint8, enum]
            %       0x00 = Enabled
            %       0x01 = Disabled
            %       0x02 = Failure
            
            % Struct unpacker
            str = Struct(server.get_rx_data());
            
            % Unpack numeric data
            arr = zeros(14, 1);
            for i = 1:length(arr)
                arr(i) = str.get('single');
            end
            obj.q = Quat(arr(1:4));
            obj.w = arr(5:7);
            obj.accel = obj.q.rotate(arr(8:10));
            obj.forces = arr(11:14);
            
            % Unpack state
            state_byte = str.get('uint8');
            switch (state_byte)
                case 0, obj.state = 'Enabled';
                case 1, obj.state = 'Disabled';
                case 2, obj.state = 'Failure';
                otherwise, obj.state = 'INVALID';
            end
            
            % Set RX flag
            obj.got_rx = true;
        end
    end
end