% Task 6: Real-Time Step Response Logger
% Mechatronics Lab, Session 6 - University of Tehran
%
% Logs the open-loop step response sent by
% Arduino/lab6_task6_step_response (one RPM value per line at 100 Hz),
% plots it in the time and frequency domains, and saves it to
% motor_step_response.mat and motor_step_response.csv.
%
% Requires MATLAB R2019b or newer (serialport).

clear; clc; close all;

% Configuration
COM_PORT = "COM3";    % Change to your Arduino port
BAUDRATE = 115200;
NUM_SAMPLES = 200;    % 2 seconds at 100 Hz
SAMPLING_FREQ = 100;  % Hz

% Open the serial port
s = serialport(COM_PORT, BAUDRATE);
configureTerminator(s, "LF");
flush(s);  % Clear any old data

fprintf('Connected to Arduino on %s\n', COM_PORT);
fprintf('Logging %d samples at %d Hz...\n', NUM_SAMPLES, SAMPLING_FREQ);

% Pre-allocate
rpm = zeros(NUM_SAMPLES, 1);
timeVec = (0:NUM_SAMPLES-1) / SAMPLING_FREQ;

% Collect data
i = 1;
tic;
while i <= NUM_SAMPLES
    if s.NumBytesAvailable > 0
        try
            % Named rxLine (not "line") so the line() plotting call below
            % still works
            rxLine = readline(s);
            value = str2double(strtrim(rxLine));

            % Keep only numeric lines
            if ~isnan(value)
                rpm(i) = value;
                i = i + 1;

                % Progress indicator
                if mod(i, 50) == 0
                    fprintf('Collected %d / %d samples\n', i, NUM_SAMPLES);
                end
            end
        catch
            % Skip corrupted lines
            continue;
        end
    end
end
elapsedTime = toc;
fprintf('Data collection complete in %.2f seconds\n', elapsedTime);

% Close the serial port
clear s;

% Plot results
figure('Position', [100, 100, 1200, 500]);

% Time domain
subplot(1,2,1);
plot(timeVec, rpm, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Motor Speed (RPM)', 'FontSize', 12);
title('Step Response (Time Domain)', 'FontSize', 14, 'FontWeight', 'bold');

% Steady-state line (mean of the last 51 samples)
steadyState = mean(rpm(end-50:end));
line([0 max(timeVec)], [steadyState steadyState], ...
    'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
text(max(timeVec)*0.7, steadyState+30, ...
    sprintf('Steady State: %.1f RPM', steadyState), ...
    'FontSize', 10, 'Color', 'r');

% Frequency domain (FFT)
subplot(1,2,2);
N = length(rpm);
f = (0:N-1) * (SAMPLING_FREQ/N);
Y = fft(rpm);
P = abs(Y/N);
P = P(1:N/2+1);
P(2:end-1) = 2*P(2:end-1);
f = f(1:N/2+1);

semilogx(f, 20*log10(P), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Magnitude (dB)', 'FontSize', 12);
title('Step Response (Frequency Domain)', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0.1 50]);

% Save data
save('motor_step_response.mat', 'rpm', 'timeVec', 'steadyState', 'SAMPLING_FREQ');
fprintf('Data saved to motor_step_response.mat\n');

% Export to CSV
dataTable = table(timeVec', rpm, 'VariableNames', {'Time_s', 'RPM'});
writetable(dataTable, 'motor_step_response.csv');
fprintf('Data exported to motor_step_response.csv\n');
