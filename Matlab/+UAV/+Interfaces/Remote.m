classdef Remote < UAV.Interfaces.Interface
    %REMOTE Bluetooth remote control for UAV
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected, Constant)
        start_byte = hex2dec('FF');     % Msg start byte
        msg_id_state = hex2dec('AA');   % State cmd ID
        msg_id_params = hex2dec('BB');  % Param msg ID
        msg_id_update = hex2dec('CC');  % Update msg ID
        timeout = 0.5;                  % Bluetooth timeout [s]
    end
    
    properties (Access = protected)
        server;     % Serial interface [SerialServer]
        got_state;  % Response flag [logical]
        uav_cmd;    % UAV command [UAV.State.Cmd]
    end
    
    methods (Access = public)
        function obj = Remote(model, params, bt_name)
            %obj = REMOTE(model, params, bt_name) Construct UAV remote
            %   Inputs:
            %       model = UAV model [UAV.Model]
            %       params = Flight params [UAV.Params]
            %       bt_name = Device Bluetooth name [char]
            
            % Default args
            if nargin < 3, bt_name = 'UavBot'; end
            if nargin < 2, params = UAV.Params(); end
            if nargin < 1, model = UAV.Model(); end
            
            % Superconstructor
            obj = obj@UAV.Interfaces.Interface(model, params);

            % Set up serial server
            bluetooth_ = make_bluetooth(bt_name);
            obj.server = SerialServer(bluetooth_, obj.start_byte);
            obj.server.add_tx(obj.msg_id_state, 1, @obj.msg_tx_state);
            obj.server.add_tx(obj.msg_id_params, 56, @obj.msg_tx_params);
            obj.server.add_tx(obj.msg_id_update, 20, @obj.msg_tx_update);
            obj.server.add_rx(obj.msg_id_update, 57, @obj.msg_rx_update);
            
            % Init fields
            obj.got_state = false;
            obj.uav_cmd = [];
            
            % Set params on UAV
            obj.server.tx(obj.msg_id_params);
        end
        
        function state = update(obj, cmd)
            %state = UPDATE(obj, cmd)
            %   Send commands and get new state
            %   Inputs:
            %       cmd = UAV command [UAV.State.Cmd]
            %   Outputs:
            %       state = UAV state [UAV.State.State]
            
            % Transmit commands
            obj.uav_cmd = cmd;
            if obj.state.enum ~= obj.uav_cmd.enum
                obj.server.tx(obj.msg_id_state);
            end
            obj.server.tx(obj.msg_id_update);
            
            % Wait for response
            obj.got_state = false;
            timer = Timer();
            while ~obj.got_state
                obj.server.rx();
                if timer.elapsed(obj.timeout)
                    error('Bluetooth timed out.');
                end
            end
            
            % Return state
            state = obj.state;
        end
        
        function set_params(obj, params)
            %SET_PARAMS(obj, params)
            %   Set flight parameters
            %   Inputs:
            %       params = Flight params [UAV.Params]
            obj.params = params;
            obj.server.tx(obj.msg_id_params);
        end
        
        function delete(obj)
            %DELETE(obj) Disonnects from Bluetooth
            fclose(obj.server.get_serial());
        end
    end
    
    methods (Access = protected)
        function msg_tx_state(obj, server)
            %MSG_TX_STATE(obj, server) Packs State TX message
            %   Data format:
            %   - State enum cmd [uint8]
            server.set_tx_data(obj.uav_cmd.enum);
        end
        
        function msg_tx_params(obj, server)
            %MSG_TX_PARAMS(obj, server) Packs params TX message
            %   Data format:
            %   - Min linear throttle [float, [0, 1]]
            %   - Max linear throttle [float, [0, 1]]
            %   - Quat-x P-gain [thr/rad]
            %   - Quat-x I-gain [thr/(rad*s)]
            %   - Quat-x D-gain [thr/(rad/s)]
            %   - Quat-x feed-forward [thr]
            %   - Quat-y P-gain [thr/rad]
            %   - Quat-y I-gain [thr/(rad*s)]
            %   - Quat-y D-gain [thr/(rad/s)]
            %   - Quat-y feed-forward [thr]
            %   - Quat-z P-gain [thr/rad]
            %   - Quat-z I-gain [thr/(rad*s)]
            %   - Quat-z D-gain [thr/(rad/s)]
            %   - Quat-z feed-forward [thr]
            str = Struct();
            str.set(obj.params.thr_min, 'single');
            str.set(obj.params.thr_max, 'single');
            str.set(obj.params.qx_kp, 'single');
            str.set(obj.params.qx_ki, 'single');
            str.set(obj.params.qx_kd, 'single');
            str.set(obj.params.qx_ff, 'single');
            str.set(obj.params.qy_kp, 'single');
            str.set(obj.params.qy_ki, 'single');
            str.set(obj.params.qy_kd, 'single');
            str.set(obj.params.qy_ff, 'single');
            str.set(obj.params.qz_kp, 'single');
            str.set(obj.params.qz_ki, 'single');
            str.set(obj.params.qz_kd, 'single');
            str.set(obj.params.qz_ff, 'single');
            server.set_tx_data(str.get_buffer());
        end
        
        function msg_tx_update(obj, server)
            %MSG_TX_UPDATE(obj, server) Packs Update TX message
            %   Data format:
            %   - Angular position cmd [float, [w; x; y; z]]
            %   - Linear throttle cmd [float]
            str = Struct();
            str.set(obj.uav_cmd.ang_pos.vector(), 'single');
            str.set(obj.uav_cmd.thr_lin, 'single');
            server.set_tx_data(str.get_buffer());
        end
        
        function msg_rx_update(obj, server)
            %MSG_RX_UPDATE(obj, server) Unpacks Update RX message
            %   Data format:
            %   - Angular position [float, [w; x; y; z]]
            %   - Angular velocity [float, [x; y; z], rad/s]
            %   - Local acceleration [float, [x; y; z], m/s^2]
            %   - Prop throttles [float, [++, +-, -+, --], [0, 1]]
            %   - State enum cmd [uint8]

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
            thr_props = arr(11:14);
            
            % Unpack enum
            import('UAV.State.Enum');
            enum_byte = str.get('uint8');
            switch (enum_byte)
                case Enum.Enabled, enum = Enum.Enabled;
                case Enum.Disabled, enum = Enum.Disabled;
                case Enum.Failed, enum = Enum.Failed;
            end
            
            % Update state and response flag
            obj.state = UAV.State.State(ang_pos, ang_vel, lin_acc, thr_props, enum);
            obj.got_state = true;
        end
    end
end

