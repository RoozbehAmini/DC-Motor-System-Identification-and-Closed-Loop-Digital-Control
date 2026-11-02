%% Automated Compensator Design
% Mechatronics Lab, Session 7 - University of Tehran (Fall 2025)
%
% Designs the three speed controllers with root locus angle/magnitude
% conditions, discretizes them with the Tustin method at 200 Hz, and
% prints the difference-equation coefficients for
% Arduino/lab7_closed_loop_control.
%
% If the required phase is larger than 60 deg, two identical stages are
% cascaded, each supplying half of the phase.
%
% Requires the Control System Toolbox.

clc; close all;
G_s = tf(113.2, [1 18.05]);  % Identified plant
Ts = 1/200;                  % 200 Hz sampling

fprintf('==============================================\n');
fprintf('MECHATRONICS LAB - CONTROLLER DESIGN\n');
fprintf('Plant: G(s) = 113.2 / (s + 18.05)\n');
fprintf('Sampling: 200 Hz (Ts = 5 ms)\n');
fprintf('==============================================\n\n');

% Controller 1
fprintf('>>> CONTROLLER 1: Zeta=0.4, Wn=33\n');
[C1, T1] = design_compensator_final(G_s, 0.4, 33);
controller2diffeq(C1, Ts);
info1 = stepinfo(feedback(C1*G_s, 1));
disp(info1);

% Controller 2
fprintf('\n>>> CONTROLLER 2: Zeta=0.75, Wn=42\n');
[C2, T2] = design_compensator_final(G_s, 0.75, 42);
controller2diffeq(C2, Ts);
info2 = stepinfo(feedback(C2*G_s, 1));
disp(info2);

% Controller 3
fprintf('\n>>> CONTROLLER 3: Zeta=0.24, Wn=99\n');
[C3, T3] = design_compensator_final(G_s, 0.24, 99);
controller2diffeq(C3, Ts);
info3 = stepinfo(feedback(C3*G_s, 1));
disp(info3);

fprintf('\n==============================================\n');
fprintf('Design complete. Paste coefficients to Arduino.\n');
fprintf('==============================================\n');

%% Main design function
function [C_s, CL_sys] = design_compensator_final(G_s, zeta_req, wn_req)
    if nargin < 3
        error('Usage: design_compensator_final(G, zeta, wn)');
    end

    % 1. Target pole location
    sigma = zeta_req * wn_req;
    wd = wn_req * sqrt(1 - zeta_req^2);
    s_d = -sigma + 1i*wd;

    fprintf('\n========== COMPENSATOR DESIGN ==========\n');
    fprintf('Target Pole: %.2f +/- j%.2f\n', real(s_d), imag(s_d));

    % 2. Plant angle at s_d
    [z_p, p_p, ~] = zpkdata(G_s, 'v');
    angle_G = 0;
    for i = 1:length(z_p)
        angle_G = angle_G + rad2deg(angle(s_d - z_p(i)));
    end
    for i = 1:length(p_p)
        angle_G = angle_G - rad2deg(angle(s_d - p_p(i)));
    end

    % 3. Required phase contribution
    phi_C = -180 - angle_G;

    % Normalize to (-180, 180]
    while phi_C <= -180, phi_C = phi_C + 360; end
    while phi_C > 180,   phi_C = phi_C - 360; end

    fprintf('Plant Angle: %.2f deg\n', angle_G);
    fprintf('Required Phase (phi): %.2f deg\n', phi_C);

    % 4. Two-stage logic (threshold: 60 deg)
    if abs(phi_C) > 60
        fprintf('Large phase requirement (|phi| > 60 deg).\n');
        fprintf('Strategy: TWO-STAGE compensator (design for phi/2, then square).\n');

        phi_stage = phi_C / 2;

        % One stage for half the angle, then square it
        [C_stage, zc, pc] = design_single_stage(s_d, phi_stage, sigma, wd);
        C_s_raw = C_stage * C_stage;

        fprintf('Single Stage: Zero=%.4f, Pole=%.4f (contributes %.2f deg)\n', ...
            zc, pc, phi_stage);
        fprintf('Full Compensator: [Stage]^2 (total %.2f deg)\n', phi_C);
    else
        fprintf('Single-stage compensator sufficient.\n');
        [C_s_raw, zc, pc] = design_single_stage(s_d, phi_C, sigma, wd);
        fprintf('Zero: %.4f, Pole: %.4f\n', zc, pc);
    end

    % 5. Gain from the magnitude condition
    val_C = evalfr(C_s_raw, s_d);
    val_G = evalfr(G_s, s_d);
    K = 1 / abs(val_C * val_G);

    C_s = K * C_s_raw;
    fprintf('Overall Gain: %.4f\n', K);

    % 6. Closed-loop system
    CL_sys = feedback(C_s * G_s, 1);

    % 7. Plots
    figure('Name', sprintf('Controller Design: zeta=%.2f, wn=%.1f', zeta_req, wn_req), ...
           'Position', [100, 100, 1200, 800]);

    % Uncompensated root locus
    subplot(2,3,1);
    rlocus(G_s); hold on;
    plot(real(s_d), imag(s_d), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
    title('Uncompensated Root Locus');
    xlabel('Real Axis (rad/s)'); ylabel('Imaginary Axis (rad/s)');
    grid on; axis equal;
    xlim([-100 20]); ylim([-120 120]);
    legend('Root Locus', 'Desired Pole');

    % Compensated root locus
    subplot(2,3,2);
    rlocus(C_s * G_s); hold on;
    plot(real(s_d), imag(s_d), 'go', 'MarkerSize', 10, 'LineWidth', 2, ...
         'MarkerFaceColor', 'g');
    title('Compensated Root Locus');
    xlabel('Real Axis (rad/s)'); ylabel('Imaginary Axis (rad/s)');
    grid on; axis equal;
    xlim([-100 20]); ylim([-120 120]);
    legend('Root Locus', 'Desired Pole');

    % Step response
    subplot(2,3,3);
    step(CL_sys, 0.5);
    title(sprintf('Step Response (\\zeta=%.2f, \\omega_n=%.1f)', zeta_req, wn_req));
    xlabel('Time (s)'); ylabel('Amplitude');
    grid on;

    % Bode plot
    subplot(2,3,4:6);
    bode(CL_sys);
    title('Frequency Response (Bode Plot)');
    grid on;
end

%% Single-stage design helper
function [C_stage, zc, pc] = design_single_stage(s_d, phi, sigma, wd)
    s = tf('s');

    if phi > 0
        % Lead compensator
        theta = rad2deg(angle(s_d));
        gamma = 0.5 * phi;

        angle_z = (180 - theta) + gamma;
        angle_p = (180 - theta) - gamma;

        zc = sigma + wd / tan(deg2rad(angle_z));
        pc = sigma + wd / tan(deg2rad(angle_p));

        if pc < 0
            warning('Pole < 0 (RHP). Adjusting to avoid instability.');
            pc = 0.1;
        end
    else
        % Lag compensator
        pc = 0.1 * sigma;
        if pc < 0.01, pc = 0.01; end

        angle_p_sd = rad2deg(angle(s_d + pc));
        angle_z_sd = angle_p_sd + phi;

        zc = (imag(s_d) / tan(deg2rad(angle_z_sd))) - real(s_d);

        if zc < pc
            zc = pc * 1.1;
        end
    end

    C_stage = (s + zc) / (s + pc);
end

%% Discretization function
function [difference_eq, A, B, K] = controller2diffeq(D_s, Ts, method)
    if nargin < 2
        error('Sampling time Ts must be specified');
    end
    if nargin < 3
        method = 'tustin';
    end

    % Discretize (Tustin / bilinear by default)
    D_z = c2d(D_s, Ts, method);

    % Coefficients
    [num_z, den_z] = tfdata(D_z, 'v');

    % Normalize so the denominator starts with 1
    if den_z(1) ~= 1
        num_z = num_z / den_z(1);
        den_z = den_z / den_z(1);
    end

    B = num_z;         % Numerator coefficients
    A = den_z(2:end);  % Denominator coefficients (excluding a0 = 1)

    % Build the difference equation string
    terms_in = {};
    for i = 1:length(B)
        if abs(B(i)) > 1e-9
            if i == 1
                terms_in{end+1} = sprintf('%.6f*e[k]', B(i));
            else
                terms_in{end+1} = sprintf('%.6f*e[k-%d]', B(i), i-1);
            end
        end
    end

    terms_out = {};
    for i = 1:length(A)
        if abs(A(i)) > 1e-9
            terms_out{end+1} = sprintf('%.6f*u[k-%d]', -A(i), i);
        end
    end

    all_terms = [terms_in, terms_out];
    difference_eq = ['u[k] = ', strjoin(all_terms, ' + ')];
    difference_eq = strrep(difference_eq, '+ -', '- ');

    % Inverse DC gain (returned for reference)
    dc_val = dcgain(D_s);
    if isinf(dc_val) || isnan(dc_val) || abs(dc_val) < 1e-6
        K = 1.0;
    else
        K = 1.0 / dc_val;
    end

    % Pad to second order so the Arduino printout also works for a
    % single-stage (first-order) controller
    Bp = [B, zeros(1, 3 - numel(B))];
    Ap = [A, zeros(1, 2 - numel(A))];

    % Display results
    fprintf('\n========== DISCRETIZATION ==========\n');
    fprintf('Continuous Controller:\n');
    D_s
    fprintf('Method: %s, Ts = %.4f s (%.0f Hz)\n', method, Ts, 1/Ts);
    fprintf('\nDiscrete Transfer Function:\n');
    D_z
    fprintf('\nDIFFERENCE EQUATION:\n');
    fprintf('%s\n', difference_eq);
    fprintf('\nCOEFFICIENTS FOR ARDUINO:\n');
    fprintf('float b0 = %.6f, b1 = %.6f, b2 = %.6f;\n', Bp(1), Bp(2), Bp(3));
    fprintf('float a1 = %.6f, a2 = %.6f;\n', Ap(1), Ap(2));
    fprintf('========================================\n');
end
