close all

% Provide time frame in seconds
senseFrame = 60; 
sample_rate = 1/20;
% Time vector
tVector = [0:sample_rate:senseFrame];


subplot(4, 1, 1)
hold on
% Create handle to Azimuth animatedline object
hAzimuth = animatedline('color', 'r', 'linewidth', 1.25);
% Create handle to Pitch animatedline object
hPitch = animatedline('color', 'k', 'linewidth', 1.25);
% Create handle to Roll animatedline object
hRoll = animatedline('color', 'b', 'linewidth', 1.25);
legend('Azimuth (rad)','Pitch (rad)','Roll (rad)');
ylabel('Euler Angles (rad)');xlabel('Time (s)');
title('Reading Orientation of the ICM-20498 sensor', 'fontsize', 12);
% axis([0 senseFrame -6.5 6.5])
grid minor
hold off

subplot(4, 1, 2)
hold on
% Create handle to X-axis acceleration animatedline object
hAx = animatedline('color', 'r', 'linewidth', 1.25);
% Create handle to Y-axis acceleration animatedline object
hAy = animatedline('color', 'k', 'linewidth', 1.25);
% Create handle to Z-axis acceleration animatedline object
hAz = animatedline('color', 'b', 'linewidth', 1.25);
legend('A_x (m/s^2)','A_y (m/s^2)','A_z (m/s^2)','numcolumns',3);
ylabel('Acceleration (m/s^2)');xlabel('Time (s)');
title('Reading Accelerometer values from ICM-20498 sensor', 'fontsize', 12);
% axis([0 senseFrame -30 30]);
hold off
grid minor

subplot(4, 1, 3)
% Create handle to X-axis angular velocity animatedline object
hVx = animatedline('color', 'r', 'linewidth', 1.25);
% Create handle to Y-axis angular velocity animatedline object
hVy = animatedline('color', 'k', 'linewidth', 1.25);
% Create handle to Z-axis angular velocity animatedline object
hVz = animatedline('color', 'b', 'linewidth', 1.25);
legend('\omega_x (rad/s)','\omega_y (rad/s)','\omega_z (rad/s)');
ylabel('Angular Velocity (rad/s)');xlabel('Time (s)');
title('Reading Angular velocity values from ICM-20498 sensor', 'fontsize', 12);
axis([0 senseFrame -10 10]);
hold off
grid minor

subplot(4, 1, 4)
% Create handle to X-axis magnetic field animatedline object
hMagx = animatedline('color', 'r', 'linewidth', 1.25);
% Create handle to Y-axis magnetic field animatedline object
hMagy = animatedline('color', 'k', 'linewidth', 1.25);
% Create handle to Z-axis magnetic field animatedline object
hMagz = animatedline('color', 'b', 'linewidth', 1.5);
legend('\mu_x (\muT)','\mu_y (\muT)','\mu_z (\muT)');
ylabel('Magnetic Field (\muT)');xlabel('Time (s)');
title('Reading Magnetometer values from ICM-20498 sensor', 'fontsize', 12);
% axis([0 senseFrame -50 50]);
hold off
grid minor