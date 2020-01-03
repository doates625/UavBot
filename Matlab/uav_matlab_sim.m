function uav_matlab_sim(f_sim, q_pole, th_min, th_max, t_dur)
%UAV_MATLAB_SIM(f_sim, q_pole, th_min, th_max, t_dur)
%   Run UAV Matlab simulation
%   Inputs:
%       f_sim = Sim frequency [Hz]
%       q_pole = Quat ctrl pole [s^-1]
%       th_min = Min thrust ratio [0-1]
%       th_max = Max thrust ratio [0-1]
%       t_dur = Sim duration [s]
%   Author: Dan Oates (WPI Class of 2020)
clc

% Default args
if nargin < 5, t_dur = 10; end
if nargin < 4, th_max = 0.9; end
if nargin < 3, th_min = 0.1; end
if nargin < 2, q_pole = -10; end
if nargin < 1, f_sim = 50; end

% Initial Display
fprintf('UAV Matlab Simulator\n\n')

% Simulate model
model = UavModel();
uav = UavMatlabSim(model, f_sim, q_pole, th_min, th_max);
uav_sim(uav, t_dur, true);

end