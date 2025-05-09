%% Modified Lidar Simulation: Target Flat on the Ground
clear all; clc;

% Target patch is 10 cm x 10 cm on the ground
areaOfPatch_cm2 = 100;   % 10 * 10 in cm^2
sensorHeight = 0.2;      % Lidar height from the ground in meters

%% Parameters for three different LIDARs
% OS1-128
theta_az_1 = 0.35;        % Azimuth resolution [deg]
theta_el_1 = 45/128;      % Elevation resolution [deg]
fov_h_1 = 360;            % Horizontal FoV [deg]
fov_v_1 = 45;             % Vertical FoV [deg]

% OS0-64
theta_az_2 = 0.35;      % or 0.18 if using 2048 horizontal steps
theta_el_2 = 90.8/64;   % ~1.42 deg
fov_h_2 = 360;
fov_v_2 = 90.8;

% HDL64E
theta_az_3 = 0.08;
theta_el_3 = 26.8/64;
fov_h_3 = 360;
fov_v_3 = 26.8;

%% Convert angles from degrees to radians
theta_az_rad_1 = deg2rad(theta_az_1);
theta_el_rad_1 = deg2rad(theta_el_1);
fov_v_rad_1 = deg2rad(fov_v_1);

theta_az_rad_2 = deg2rad(theta_az_2);
theta_el_rad_2 = deg2rad(theta_el_2);
fov_v_rad_2 = deg2rad(fov_v_2);

theta_az_rad_3 = deg2rad(theta_az_3);
theta_el_rad_3 = deg2rad(theta_el_3);
fov_v_rad_3 = deg2rad(fov_v_3);

%% Distances from 0.01 to 40 meters
distances = 0.01 : 0.01 : 40;

% Preallocate arrays for the results
r1 = zeros(size(distances));
r2 = zeros(size(distances));
r3 = zeros(size(distances));

%% Compute NoP for each distance d
for i = 1:length(distances)
    d = distances(i);

    %---------------------------
    % 1) Check if ground patch is in the vertical FoV
    % The angle from the sensor to a point on the ground at horizontal distance d
    % is alpha = arctan(sensorHeight / d).
    alpha_1 = atan2(sensorHeight, d);
    alpha_2 = alpha_1;
    alpha_3 = alpha_1;  % same geometry, just different sensor specs

    % OS1-128
    if alpha_1 <= (fov_v_rad_1 / 2)
        % Line-of-sight distance to patch center
        losDist = sqrt(d^2 + sensorHeight^2);

        % Beam footprint at line-of-sight distance
        w1 = 2 * losDist * tan(theta_az_rad_1 / 2);  % [m]
        h1 = 2 * losDist * tan(theta_el_rad_1 / 2);  % [m]

        % Convert to cm^2
        w1_cm = w1 * 100;
        h1_cm = h1 * 100;
        A1_footprint_cm2 = w1_cm * h1_cm;

        % NoP = (patch area / beam footprint area) * factor
        r1(i) = areaOfPatch_cm2 * (1 / A1_footprint_cm2) * 4;
    else
        r1(i) = 0;  % patch is out of FOV
    end

    % OS1-64
    if alpha_2 <= (fov_v_rad_2 / 2)
        losDist = sqrt(d^2 + sensorHeight^2);

        w2 = 2 * losDist * tan(theta_az_rad_2 / 2);
        h2 = 2 * losDist * tan(theta_el_rad_2 / 2);

        w2_cm = w2 * 100;
        h2_cm = h2 * 100;
        A2_footprint_cm2 = w2_cm * h2_cm;

        r2(i) = areaOfPatch_cm2 * (1 / A2_footprint_cm2) * 4;
    else
        r2(i) = 0;
    end

    % HDL64E
    if alpha_3 <= (fov_v_rad_3 / 2)
        losDist = sqrt(d^2 + sensorHeight^2);

        w3 = 2 * losDist * tan(theta_az_rad_3 / 2);
        h3 = 2 * losDist * tan(theta_el_rad_3 / 2);

        w3_cm = w3 * 100;
        h3_cm = h3 * 100;
        A3_footprint_cm2 = w3_cm * h3_cm;

        r3(i) = areaOfPatch_cm2 * (1 / A3_footprint_cm2) * 4;
    else
        r3(i) = 0;
    end
end

%% Plot Results on a Semilog-y scale
figure('Color',[1,1,1]);
semilogy(distances, r1, 'k-', 'LineWidth', 2);  hold on;
semilogy(distances, r2, 'k--','LineWidth', 2);
semilogy(distances, r3, 'k:','LineWidth', 2);

% Horizontal line at NoP = 4
yline(4, 'k-', 'NoP = 4', 'LabelVerticalAlignment','top', ...
       'LabelHorizontalAlignment','right', 'LineWidth',2);

xlabel('Horizontal Distance d (m)', 'FontWeight','bold', 'FontSize',12);
ylabel('NoP for 10 cm x 10 cm Ground Patch', 'FontWeight','bold','FontSize',12);
xlim([0, 40]); ylim([1, 1e4]);
title(sprintf('Flat Patch at Ground Level (h = %.1f m)', sensorHeight), ...
      'FontWeight','bold', 'FontSize', 12);
legend('OS1-128','OS0','HDL64E','Location','Best');
grid on; hold off;
