classdef Remote < UAV.Interfaces.Interface
    %REMOTE Bluetooth remote control for UAV
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte
        msg_id_state = hex2dec('00');   % State cmd ID
        msg_id_update = hex2dec('01');  % Update msg ID
    end
    
    properties (Access = protected)
        server;     % Serial interface [SerialServer]
        got_resp;   % Response flag [logical]
        cmd;        % UAV command [UAV.Cmd]
    end
    
    methods (Access = public)
        function obj = Remote(bt_name)
            %obj = REMOTE(bt_name) Construct UAV remote with given bluetooth name [char]
            
            % Default args
            if nargin < 1
                bt_name = 'UavBot';
            end
            
            % Superconstructor
            obj = obj@UAV.Interfaces.Interface();

            % Set up serial server
            bluetooth_ = make_bluetooth(bt_name);
            obj.server = SerialServer(bluetooth_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_state, 0, @obj.msg_tx_state);
            obj.server.add_tx(obj.msg_id_update, 16, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 57, @obj.msg_rx_update);
            
            % Init fields
            obj.got_resp = false;
            obj.cmd = [];
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   Inputs:
            %       cmd = UAV command [UAV.Cmd]
            %   Outputs:
            %       state = UAV state [UAV.State]
            
            % Transmit commands
            obj.cmd = cmd;
            if ~strcmp(obj.state.state, obj.cmd.state)
                obj.server.tx(obj.msg_id_state);
            end
            obj.server.tx(obj.msg_id_update);
            
            % Wait for response
            obj.got_resp = false;
            while ~obj.got_resp
                obj.server.rx();
            end
            
            % Return state
            state = obj.state;
        end
        
        function delete(obj)
            %DELETE(obj) Disonnects from Bluetooth
            fclose(obj.server.get_serial());
        end
    end
    
    methods (Access = protected)
        function msg_tx_state(obj, server)
            %MSG_TX_STATE(obj, server) Packs State TX message
            switch obj.cmd.state
                case 'Enabled', state_cmd = 0;
                case 'Disabled', state_cmd = 1;
                case 'Failed', state_cmd = 2;
            end
            server.set_tx_data(state_cmd);
        end
        
        function msg_tx_update(obj, server)
            %MSG_TX_UPDATE(obj, server) Packs Update TX message
            %   Data format:
            %   [00-03] Global accel x [single, m/s^2]
            %   [04-07] Global accel y [single, m/s^2]
            %   [08-11] Global accel z [single, m/s^2]
            %   [12-15] Heading cmd [single, rad]
            str = Struct();
            str.set(obj.cmd.acc, 'single');
            str.set(obj.cmd.tz, 'single');
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
            %       0x02 = Failed
            
            % Struct unpacker
            str = Struct(server.get_rx_data());
            
            % Unpack numeric data
            arr = zeros(14, 1);
            for i = 1:length(arr)
                arr(i) = str.get('single');
            end
            ang_pos = Quat(arr(1:4));
            ang_vel = arr(5:7);
            lin_acc = ang_pos.rotate(arr(8:10));
            f_prop = arr(11:14);
            
            % Unpack state
            state_byte = str.get('uint8');
            switch (state_byte)
                case 0, state = 'Enabled';
                case 1, state = 'Disabled';
                case 2, state = 'Failed';
            end
            
            % Set state and response flag
            obj.state = UAV.State(ang_pos, ang_vel, lin_acc, f_prop, state);
            obj.got_resp = true;
        end
    end
end

