function uav_matlab_pilot(q_pole, th_min, th_max, acc_max, wz_max)
%UAV_MATLAB_PILOT(q_pole, th_min, t_max, acc_max, wz_max)
%   Run UAV Matlab pilot simulation
%   Inputs:
%       q_pole = Quat ctrl pole [s^-1]
%       th_min = Min thrust ratio [0-1]
%       th_max = Max thrust ratio [0-1]
%       acc_max = Max x-y-z accel cmd [m/s^2]
%       wz_max = Max yaw velocity cmd [rad/s]
%   Author: Dan Oates (WPI Class of 2020)

% Params and defaults
if nargin < 5, wz_max = pi/4; end
if nargin < 4, acc_max = 1.0; end
if nargin < 3, th_max = 0.9; end
if nargin < 2, th_min = 0.1; end
if nargin < 1, q_pole = -5; end
frame_rate = 50;

% Create and pilot model
model = UavModel();
uav = UavMatlabSim(model, frame_rate, q_pole, th_min, th_max);
uav_pilot(uav, acc_max, wz_max);

end