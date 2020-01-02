function uav_pilot(uav, acc_max, wz_max)
%UAV_PILOT(name, acc_max, wz_max) Pilot UAV with Xbox controller
%   Inputs:
%       uav = UAV interface [UAVInterface]
%       acc_max = Max x-y-z accel cmd [m/s^2]
%       wz_max = Max yaw velocity cmd [rad/s]
%   Author: Dan Oates (WPI Class of 2020)
clc

% Default args
if nargin < 3, wz_max = pi/4; end
if nargin < 2, acc_max = 1.0; end

% Initial display
fprintf('UAV Pilot Program\n\n')

% Init plotter
fig = figure(1);
uav_plot = UavPlot(fig);

% Init xbox and cmds
xbox = Xbox360(1, 0.075);
acc_cmd = zeros(3, 1);
tz_int = Integrator();
real_uav = isa(uav, 'UavRemote');

% Command loop
frame_cnt = 1;
frame_rate = nan;
timer = Timer();
while true

    % Parse cmds from xbox
    if real_uav
        if xbox.btn('Start')
            % TODO Add edge detection to game controller class
            uav.set_enabled(true);
        end
        if xbox.btn('Back')
            uav.set_enabled(false);
        end
    end
    acc_cmd(1) = -acc_max * xbox.axis('Ly');
    acc_cmd(2) = -acc_max * xbox.axis('Lx');
    acc_cmd(3) = -acc_max * xbox.axis('Trig');
    tz_cmd = tz_int.update(-wz_max * xbox.axis('Rx'));
    acc_cmd = Quat([0; 0; 1], tz_cmd).rotate(acc_cmd);
    
    % Send cmds and get state
    [q, w, acc, tz, f] = uav.update(acc_cmd, tz_cmd);
    if real_uav && frame_cnt == 1
        tz_int.set(tz);
    end
    
    % Print status
    clc
    fprintf('UAV Pilot Program\n')
    if real_uav
        fprintf('State: %s\n', uav.get_state())
    end
    fprintf('\nControls\n')
    fprintf('Accel-x: Cmd = %+.2f, Act = %+.2f\n', acc_cmd(1), acc(1));
    fprintf('Accel-y: Cmd = %+.2f, Act = %+.2f\n', acc_cmd(2), acc(2));
    fprintf('Accel-z: Cmd = %+.2f, Act = %+.2f\n', acc_cmd(3), acc(3));
    fprintf('Theta-z: Cmd = %+.2f, Act = %+.2f\n', tz_cmd, tz);
    fprintf('\nForces:\n')
    fprintf('F++: %.2f\n', f(1));
    fprintf('F+-: %.2f\n', f(2));
    fprintf('F-+: %.2f\n', f(3));
    fprintf('F--: %.2f\n', f(4));
    fprintf('\nFramerate:\n%.1f\n', frame_rate)
    
    % Update live plot
    uav_plot.update(q, w);
    drawnow
    
    % Exit button
    if xbox.btn('B')
        fprintf('\nStop by user.\n\n')
        break
    end
    
    % Frame count
    frame_cnt = frame_cnt + 1;
    frame_rate = frame_cnt / timer.toc();
end

% Disable real UAVs
if real_uav
    uav.set_enabled(false);
end

end