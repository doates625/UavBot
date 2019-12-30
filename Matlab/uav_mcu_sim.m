function uav_mcu_sim(f_sim, t_dur, name, port)
%UAV_MCU_SIM(f_sim, t_dur, name, port)
%   Run UAV embedded simulation
%   
%   Inputs:
%       f_sim = Sim frequency [Hz]
%       t_dur = Sim duration [s]
%       name = Bluetooth name [char]
%       port = Serial port name [char]
%   
%   Author: Dan Oates (WPI Class of 2020)
clc, instrreset

% Default args
if nargin < 4, port = 'COM17'; end
if nargin < 3, name = 'UavBot'; end
if nargin < 2, t_dur = 10; end
if nargin < 1, f_sim = 50; end

% Initial Display
fprintf('UAV MCU Simulator\n\n')

% Create Bluetooth remote
fprintf('Connecting to Bluetooth...\n')
remote = UavRemote(name);

% Create serial simulator
fprintf('Connecting to serial...\n')
model = UavModel();
uav = UavMcuSim(model, f_sim, remote, port);

% Run simulator
uav_sim(uav, t_dur, true);

end