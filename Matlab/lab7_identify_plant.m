% System Identification: First-Order Motor Model
% Mechatronics Lab, Session 7 - University of Tehran
%
% Fits G(s) = K / (tau*s + 1) to an open-loop step response with tfest.
% The input is a step of height STEP_PWM, so G relates the PWM value
% directly to motor speed in RPM.
%
% Expects DATA_FILE to contain a vector "rpm" sampled every Ts seconds.
% The values below are the ones given in the Lab 7 report (PWM 200 step,
% 200 Hz). Change them to match your own recording.
%
% Requires the System Identification Toolbox.

clc; close all;

% Settings
DATA_FILE = 'step_response_data.mat';
Ts = 0.005;       % Sample time (s)
STEP_PWM = 200;   % PWM value of the step input

% Load the experimental data
S = load(DATA_FILE);
rpm = S.rpm(:);

% Create the iddata object (input: constant step of height STEP_PWM)
data = iddata(rpm, STEP_PWM*ones(size(rpm)), Ts, 'Name', 'mydata');
data.InputName = 'PWM';
data.OutputName = 'Speed (RPM)';

% Estimate a first-order transfer function (1 pole, 0 zeros)
sys_options = tfestOptions('Display', 'on', 'EnforceStability', true);
G_identified = tfest(data, 1, 0, sys_options);

% Extract parameters. tfest returns G(s) = num(end) / (s + den(end)),
% so the DC gain is num(end)/den(end) and tau = 1/den(end).
[num, den] = tfdata(G_identified, 'v');
K = num(end) / den(end);
tau = den(end-1) / den(end);

fprintf('Identified Plant: G(s) = %.2f / (s + %.2f)\n', num(end), den(end));
fprintf('                       = %.2f / (%.4fs + 1)\n', K, tau);
