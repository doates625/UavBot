%% Clear Workspace
clear, close, clc

%% Settings
f_sim = 50.0;   % Sim frequency [Hz]
s_q = -20.0;    % Quat ctrl pole [s^-1]

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

% Commands
acc_cmd_log = zeros(3, N);
tz_cmd_log = zeros(1, N);
acc_cmd_log(1, t_log >= 0) = +5;
acc_cmd_log(1, t_log >= 2) = -5;
acc_cmd_log(1, t_log >= 4) = 0;
acc_cmd_log(2, t_log >= 4) = +5;
acc_cmd_log(2, t_log >= 6) = -5;
acc_cmd_log(2, t_log >= 8) = 0;
tz_cmd_log(t_log >= 0) = sin(t_log);

% Simulator
model = UAVModel();
sim = UAVSim(model, f_sim, s_q);

% Run Simulator
for i = 1:N
    
    % Run simulator
    acc_cmd = acc_cmd_log(:,i);
    tz_cmd = tz_cmd_log(i);
    [q, w, f, acc, tz, q_cmd] = sim.update(acc_cmd, tz_cmd);    
    
    % Log results
    q_log(:,i) = q.vector();
    w_log(:,i) = w;
    f_log(:,i) = f;
    acc_log(:,i) = acc;
    tz_log(i) = tz;
    q_cmd_log(:,i) = q_cmd.vector();
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

%% Plot Angular Velocity
figure
names = {'x', 'y', 'z'};
for i = 1:3
    subplot(3, 1, i)
    hold on, grid on
    title(['Omega-' names{i}])
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    plot(t_log, w_log(i,:), 'b-')
    if max(w_log(i,:)) - min(w_log(i,:)) < 1e-10
        ylim([-1, +1])
    end
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
    acc_cmd_zero = max(acc_cmd_log(i,:)) - min(acc_cmd_log(i,:)) < 1e-10;
    acc_zero = max(acc_log(i,:)) - min(acc_log(i,:)) < 1e-10;
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