% Load the telemetry data from the text file
data = dlmread('pid.txt', '\t');

% Extract columns into variables
time = data(:,1);
left_speed = data(:,2);
right_speed = data(:,3);
left_setpoint = data(:,4);
right_setpoint = data(:,5);

% Compute maximum setpoint for scaling
max_left_setpoint = max(left_setpoint);
max_right_setpoint = max(right_setpoint);

% Create a new figure
figure;

% First subplot: Left motor
subplot(2,1,1);
plot(time, left_setpoint, 'r', time, left_speed, 'b');
legend('Left Setpoint', 'Actual Speed');
xlabel('Time (s)');
ylabel('Speed (ticks per 25ms)');
title('Left PID Response');
% Set y-axis limits with 10% padding, starting at 0
if max_left_setpoint > 0
    ylim([0, max_left_setpoint * 1.1]);
else
    ylim([0, 10]); % Default range if setpoint is zero
end

% Second subplot: Right motor
subplot(2,1,2);
plot(time, right_setpoint, 'r', time, right_speed, 'b');
legend('Right Setpoint', 'Actual Speed');
xlabel('Time (s)');
ylabel('Speed (ticks per 25ms)');
title('Right PID Response');
% Set y-axis limits with 10% padding, starting at 0
if max_right_setpoint > 0
    ylim([0, max_right_setpoint * 1.1]);
else
    ylim([0, 10]); % Default range if setpoint is zero
end