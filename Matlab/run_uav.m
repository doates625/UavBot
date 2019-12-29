%% Clear Workspace
clear, close, clc

%% Settings
f_sim = 50.0;   % Sim frequency [Hz]
q_pole = -30;   % Quat ctrl pole [s^-1]
th_min = 0.1;   % Min thrust ratio [0-1]
th_max = 0.9;   % Max thrust ratio [0-1]

%% Simulation

% Time Logging
t_dur = 10.0;
t_sim = 1 / f_sim;
t_log = 0 : t_sim : t_dur;
N = length(t_log);

% State Logging
q_log = zeros(4, N);
w_log = zeros(3, N);
f_log = zeros(4, N);
acc_log = zeros(3, N);
tz_log = zeros(1, N);
q_cmd_log = zeros(4, N);
alp_cmd_log = zeros(3, N);

% Commands
acc_cmd_log = zeros(3, N);
tz_cmd_log = zeros(1, N);
acc_cmd_log(1, t_log >= 0) = +5;
acc_cmd_log(1, t_log >= 2) = -5;
acc_cmd_log(1, t_log >= 4) = 0;
acc_cmd_log(2, t_log >= 4) = +5;
acc_cmd_log(2, t_log >= 6) = -5;
acc_cmd_log(2, t_log >= 8) = 0;
acc_cmd_log(3, t_log >= 3) = 5;
acc_cmd_log(3, t_log >= 5) = -5;
acc_cmd_log(3, t_log >= 7) = 0;
tz_cmd_log(t_log >= 0) = 0; % pi; % t_log;

% Simulator
model = UAVModel();
sim = UAVSim(model, f_sim, q_pole, th_min, th_max);

% Run Simulator
tz_cmd_log = wrap(tz_cmd_log, -pi, +pi);
for i = 1:N
    
    % Run simulator
    acc_cmd = acc_cmd_log(:,i);
    tz_cmd = tz_cmd_log(i);
    [q, w, f, acc, tz, q_cmd, alp_cmd, running] = sim.update(acc_cmd, tz_cmd);    
    
    % Log results
    q_log(:,i) = q.vector();
    w_log(:,i) = w;
    f_log(:,i) = f;
    acc_log(:,i) = acc;
    tz_log(i) = tz;
    q_cmd_log(:,i) = q_cmd.vector();
    alp_cmd_log(:,i) = alp_cmd;
    
    % Check for failure
    if ~running
        N = i;
        t_log = t_log(1:N);
        t_dur = t_log(end);
        q_log = q_log(:,1:N);
        w_log = w_log(:,1:N);
        f_log = f_log(:,1:N);
        acc_log = acc_log(:,1:N);
        tz_log = tz_log(1:N);
        q_cmd_log = q_cmd_log(:,1:N);
        alp_cmd_log = alp_cmd_log(:,1:N);
        acc_cmd_log = acc_cmd_log(:,1:N);
        tz_cmd_log = tz_cmd_log(:,1:N);
        disp('FAILURE')
        break
    end
end

%% Video Playback

view_az = -20;  % Plot camera azimuth [deg]
view_el = +35;  % Plot camera elevation [deg]

figure
hold on, grid on
title('UAV Pose')
xlabel('x')
ylabel('y')
zlabel('z')
plot_q_act = FramePlot3D(1, 'r-', 'g-', 'b-');
plot_q_cmd = FramePlot3D(1, 'r--', 'g--', 'b--');
plot_ax_err = VectorPlot3D('c--');
plot_w = VectorPlot3D('m--');
plot_alp_cmd = VectorPlot3D('k--');
axis([-1, 1, -1, 1, -1, 1])
view(view_az, view_el)
camproj perspective
axis square
for i = 1:N
    % Display time
    title(sprintf('UAV Pose (t = %.2f)', t_log(i)));
    
    % Parse vectors
    q = Quat(q_log(:,i));
    q_cmd = Quat(q_cmd_log(:,i));
    alp_cmd = q.rotate(alp_cmd_log(:,i));
    w = q.rotate(w_log(:,i));
    
    % Replicate QOC
    q_err = q_cmd / q;
    if q_err.w < 0
        q_err = -q_err;
        disp('YOUCH')
    end
    ax = q_err.axis();
    ax = q.rotate(ax);
    
    % Update live plots
    plot_q_act.update(q);
    plot_q_cmd.update(q_cmd);
    plot_alp_cmd.update(alp_cmd);
    plot_w.update(w);
    plot_ax_err.update(ax);
end

%% Plot Orientation
figure
names = {'w', 'x', 'y', 'z'};
for i = 1:4
    subplot(2, 2, i)
    hold on, grid on
    title(['Quat-' names{i}])
    xlabel('Time [s]')
    ylabel('Value')
    plot(t_log, q_cmd_log(i,:), 'k--')
    plot(t_log, q_log(i,:), 'b-')
    legend('Setpt', 'Value')
    ylim([-1, +1])
end

%% Plot Acceleration
figure
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

%% Plot Heading
figure
hold on, grid on
title('Heading Control')
xlabel('Time [s]')
ylabel('Heading [rad]')
plot(t_log, tz_cmd_log, 'k--')
plot(t_log, tz_log, 'b-')
ylim([-pi, +pi])
legend('Setpt', 'Value')

%% Plot Forces
figure
names = {'++', '+-', '-+', '--'};
for i = 1:4
    subplot(2, 2, i)
    hold on, grid on
    title(['Force [' names{i} ']'])
    xlabel('Time [s]')
    ylabel('Force [N]')
    plot(t_log, f_log(i,:), 'r-')
    ylim([model.f_min, model.f_max])
end