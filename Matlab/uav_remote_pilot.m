function uav_remote_pilot(name, acc_max, wz_max)
%UAV_REMOTE_PILOT(name, acc_max, wz_max)
%   Pilot remote UAV with Xbox controller
%   Inputs:
%       name = Bluetooth name [char]
%       acc_max = Max x-y-z accel cmd [m/s^2]
%       wz_max = Max yaw velocity cmd [rad/s]
%   Author: Dan Oates (WPI Class of 2020)
clc, instrreset

% Default arguments
if nargin < 3, wz_max = pi/4; end
if nargin < 2, acc_max = 3.0; end
if nargin < 1, name = 'UavBot'; end

% Create remote and run pilot
fprintf('UAV Remote Pilot\n\n')
fprintf('Connecting to Bluetooth...\n')
uav = UavRemote(name);
uav_pilot(uav, acc_max, wz_max);

end