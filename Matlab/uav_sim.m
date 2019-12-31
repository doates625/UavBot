function uav_sim(uav, t_dur, animate)
%UAV_SIM(uav, t_dur, animate)
%   Run UAV simulation
%   Inputs:
%       uav = UAV simulator [UavSim]
%       t_dur = Sim duration [s]
%       animate = Playback animation [logical]
%   Author: Dan Oates (WPI Class of 2020)

% Default args
if nargin < 3, animate = false; end
if nargin < 2, t_dur = 10; end

% Time logging
f_sim = uav.f_sim;
t_sim = 1 / f_sim;
t_log = 0 : t_sim : t_dur;
N = length(t_log);

% State logging
q_log = zeros(4, N);
w_log = zeros(3, N);
acc_log = zeros(3, N);
tz_log = zeros(1, N);
f_log = zeros(4, N);

% Commands
acc_cmd_log = zeros(3, N);
tz_cmd_log = zeros(1, N);
acc_cmd_log(1, t_log >= 0) = +5;
acc_cmd_log(1, t_log >= 2) = -20;
acc_cmd_log(1, t_log >= 4) = 0;
acc_cmd_log(2, t_log >= 4) = +20;
acc_cmd_log(2, t_log >= 6) = -5;
acc_cmd_log(2, t_log >= 8) = 0;
acc_cmd_log(3, t_log >= 3) = 5;
acc_cmd_log(3, t_log >= 5) = -20;
acc_cmd_log(3, t_log >= 7) = 0;
tz_cmd_log(t_log >= 0) = t_log;
tz_cmd_log = wrap(tz_cmd_log, -pi, +pi);

% Run simulation
fprintf('Running simulation...\n');
prog = ProgressTracker(1);
for i = 1:N
    
    % Run simulator
    acc_cmd = acc_cmd_log(:,i);
    tz_cmd = tz_cmd_log(i);
    [q, w, acc, tz, f, stat] = uav.update(acc_cmd, tz_cmd);    
    
    % Log results
    q_log(:,i) = q.vector();
    w_log(:,i) = w;
    acc_log(:,i) = acc;
    tz_log(i) = tz;
    f_log(:,i) = f;
    
    % Update progress
    prog.update(i / N);
    
    % Check for failure
    if stat
        fprintf('UAV failure.\n')
        break
    end
end

% Crop log vectors
N = i;
t_log = t_log(1:N);
q_log = q_log(:,1:N);
w_log = w_log(:,1:N);
acc_log = acc_log(:,1:N);
tz_log = tz_log(1:N);
f_log = f_log(:,1:N);
acc_cmd_log = acc_cmd_log(:,1:N);
tz_cmd_log = tz_cmd_log(:,1:N);

% Time plots
fprintf('Generating time plots...\n')

% Plot acceleration
figure(2), clf
names = {'x', 'y', 'z'};
for i = 1:3
    subplot(3, 1, i)
    hold on, grid on
    title(['Accel-' names{i} ' Control'])
    xlabel('Time [s]')
    ylabel('Accel [m/s^2]')
    plot(t_log, acc_cmd_log(i,:), 'k--')
    plot(t_log, acc_log(i,:), 'b-')
    legend('Setpt', 'Value')
    acc_cmd_zero = max(acc_cmd_log(i,:)) - min(acc_cmd_log(i,:)) < 1e-3;
    acc_zero = max(acc_log(i,:)) - min(acc_log(i,:)) < 1e-3;
    if acc_cmd_zero && acc_zero
        ylim([-1, +1])
    end
end

% Plot heading
figure(3), clf
hold on, grid on
title('Heading Control')
xlabel('Time [s]')
ylabel('Heading [rad]')
plot(t_log, tz_cmd_log, 'k--')
plot(t_log, tz_log, 'b-')
ylim([-pi, +pi])
legend('Setpt', 'Value')

% Plot forces
figure(4), clf
names = {'++', '+-', '-+', '--'};
f_min = uav.model.f_min;
f_max = uav.model.f_max;
for i = 1:4
    subplot(2, 2, i)
    hold on, grid on
    title(['Force [' names{i} ']'])
    xlabel('Time [s]')
    ylabel('Force [N]')
    plot(t_log, f_log(i,:), 'r-')
    ylim([f_min, f_max])
end

% Animation playback
if animate
    fprintf('Running animation...\n')
    uav_plot = UavPlot();
    for i = 1:N
        q = Quat(q_log(:,i));
        w = w_log(:,i);
        t = t_log(i);
        uav_plot.update(q, w, t);
        drawnow
    end
    fprintf('Animation complete!\n')
end

% Final print gap
fprintf('\n')

end