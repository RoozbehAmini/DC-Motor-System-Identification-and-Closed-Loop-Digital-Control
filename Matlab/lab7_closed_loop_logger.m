%% Closed-Loop Step Response Logger
% Mechatronics Lab, Session 7 - University of Tehran
%
% Reads the data block printed by Arduino/lab7_closed_loop_control after
% the experiment, plots the response and calculates performance metrics
% (final speed, overshoot, 2% settling time).
%
% Requires MATLAB R2019b or newer (serialport).

clc; close all;
clear device;  % Close a port left open by a previous run

% ===== SETTINGS =====
serialPort = "COM15";  % <--- CHANGE THIS TO YOUR PORT
baudRate   = 115200;
fs         = 200;      % Sampling frequency (Hz)
duration   = 2;        % Duration (seconds)
numSamples = fs * duration;

% ===== CONNECT TO ARDUINO =====
fprintf('1. Opening serial port %s...\n', serialPort);
% Opening the port resets the Arduino (DTR), which starts the experiment
device = serialport(serialPort, baudRate);
configureTerminator(device, "CR/LF");
device.Timeout = 10;
flush(device);

% ===== WAIT FOR THE DATA BLOCK =====
% The Arduino prints a header, waits 2 s, runs for 2 s and only then sends
% the data, so skip every line up to the start marker.
fprintf('2. Arduino is resetting and running the experiment...\n');
while true
    lineStr = readline(device);
    if isempty(lineStr)
        clear device;
        error('Timed out waiting for "=== EXPERIMENTAL DATA START ===".');
    end
    if contains(lineStr, "EXPERIMENTAL DATA START")
        break;
    end
end

% ===== READ DATA =====
fprintf('3. Reading experimental data...\n');
rpmData = zeros(numSamples, 1);
nRead = 0;

for i = 1:numSamples
    lineStr = readline(device);
    if isempty(lineStr)
        warning('Timeout waiting for data at sample %d', i);
        break;
    end

    val = str2double(lineStr);
    if isnan(val)
        val = 0;  % Handle NaN readings
    end
    rpmData(i) = val;
    nRead = i;

    % Progress indicator
    if mod(i, 50) == 0
        fprintf('   Received %d / %d samples\n', i, numSamples);
    end
end

fprintf('   Data acquisition complete: %d samples received.\n', nRead);
clear device;  % Close the serial port

% ===== ANALYZE DATA =====
t = (0:numSamples-1) / fs;

% Metrics
final_speed = mean(rpmData(end-20:end));
max_speed = max(rpmData);
min_speed = min(rpmData);

% Settling time (2% criterion)
settlingBand = 0.02 * final_speed;
upperBound = final_speed + settlingBand;
lowerBound = final_speed - settlingBand;

settlingIdx = numSamples;
for i = numSamples:-1:1
    if rpmData(i) > upperBound || rpmData(i) < lowerBound
        settlingIdx = i;
        break;
    end
end
settlingTime = settlingIdx / fs;

% Overshoot
if max_speed > final_speed
    overshoot = ((max_speed - final_speed) / final_speed) * 100;
else
    overshoot = 0;
end

% ===== PLOT RESULTS =====
figure('Color', 'w', 'Position', [100, 100, 900, 600]);

plot(t, rpmData, 'b-', 'LineWidth', 1.5);
hold on;
yline(final_speed, 'r--', 'LineWidth', 1.5);
yline(upperBound, 'k:', 'LineWidth', 1);
yline(lowerBound, 'k:', 'LineWidth', 1);
xline(settlingTime, 'g--', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Speed (RPM)', 'FontSize', 12);
title('Closed-Loop Step Response', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0 duration]);

legend({'Measured Speed', ...
        sprintf('Final Value: %.1f RPM', final_speed), ...
        'Settling Band (+/-2%)', ...
        '', ...
        sprintf('Settling Time: %.3f s', settlingTime)}, ...
       'Location', 'southeast');

% Metrics text box
textStr = sprintf(['Performance Metrics:\n' ...
                   'Final Speed: %.1f RPM\n' ...
                   'Max Speed: %.1f RPM\n' ...
                   'Overshoot: %.1f %%\n' ...
                   'Settling Time: %.3f s'], ...
                  final_speed, max_speed, overshoot, settlingTime);
annotation('textbox', [0.15, 0.65, 0.3, 0.2], ...
           'String', textStr, ...
           'FitBoxToText', 'on', ...
           'BackgroundColor', 'white', ...
           'EdgeColor', 'black', ...
           'FontSize', 10);

% ===== SUMMARY AND SAVE =====
fprintf('\n=== PERFORMANCE SUMMARY ===\n');
fprintf('Final Speed: %.2f RPM\n', final_speed);
fprintf('Maximum Speed: %.2f RPM\n', max_speed);
fprintf('Minimum Speed: %.2f RPM\n', min_speed);
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Settling Time: %.3f s\n', settlingTime);
fprintf('===========================\n');

saveName = input('Enter filename to save data (or press Enter to skip): ', 's');
if ~isempty(saveName)
    save(saveName, 'rpmData', 't', 'final_speed', 'overshoot', 'settlingTime', 'fs');
    fprintf('Data saved to %s.mat\n', saveName);
end
