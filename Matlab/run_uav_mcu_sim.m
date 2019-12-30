function run_uav_mcu_sim(f_sim, t_dur, name, port)
%RUN_UAV_MCU_SIM(f_sim, t_dur, name, port)
%   Run UAV embedded simulation
%   
%   Inputs:
%       f_sim = Sim frequency [Hz]
%       t_dur = Sim duration [s]
%       name = Bluetooth name [char]
%       port = Serial port name [char]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Default args
if nargin < 4, port = 'COM17'; end
if nargin < 3, name = 'UavBot'; end
if nargin < 2, t_dur = 10; end
if nargin < 1, f_sim = 50; end

% Reset instruments
instrreset;

% Simulate model
model = UavModel();
remote = UavRemote(name);
uav = UavMcuSim(model, f_sim, remote, port);
run_uav_sim(uav, t_dur, true);

end