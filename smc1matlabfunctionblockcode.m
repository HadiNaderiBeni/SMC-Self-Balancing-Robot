function [ua_SMC, s_SMC, ua_eq_SMC, ua_r_SMC, ua_sw_SMC, theta_deg_SMC] = ...
    TwoState_SMC_Controller( ...
    x_hat_rad, CTRL_ENABLE, CTRL_RESET, ...
    A, B, S_SMC, Kr_SMC, Ksw_SMC, phi_SMC, uMax_SMC)
%#codegen
% Two-state Sliding Mode Controller for self-balancing robot.
%
% State:
%   x_hat_rad = [gamma; theta; dot_gamma; dot_theta]
%
% Units:
%   [rad; rad; rad/s; rad/s]
%
% Sliding surface:
%   s = dot_theta + lambda_theta*theta
%
% Implemented as:
%   s = S_SMC*x_hat_rad
%
% where:
%   S_SMC = [0, lambda_theta, 0, 1]
%
% Linear model:
%   x_dot = A*x + B*ua
%
% Control law:
%   ua = ua_eq + ua_r + ua_sw
%
% where:
%   ua_eq = -(S*A*x)/(S*B)
%   ua_r  = -(Kr*s)/(S*B)
%   ua_sw = -Ksw*sat(s/phi)

%% Safe output when disabled or reset
if CTRL_RESET || ~CTRL_ENABLE
    ua_SMC = 0;
    s_SMC = 0;
    ua_eq_SMC = 0;
    ua_r_SMC = 0;
    ua_sw_SMC = 0;
    theta_deg_SMC = 0;
    return;
end

%% Extract theta for monitoring
theta_SMC = x_hat_rad(2);
theta_deg_SMC = theta_SMC * 180/pi;

%% Sliding variable
s_SMC = S_SMC*x_hat_rad;

%% Compute S*B
SB_SMC = S_SMC*B;

if abs(SB_SMC) < 1e-6
    ua_eq_SMC = 0;
    ua_r_SMC = 0;
else
    %% Equivalent control
    % This cancels the nominal dynamics of s.
    %
    % s_dot = S*(A*x + B*u)
    %
    % Choose ua_eq so that:
    %   S*A*x + S*B*ua_eq = 0

    ua_eq_SMC = -(S_SMC*A*x_hat_rad)/SB_SMC;

    %% Smooth reaching term
    % Desired reaching law contribution:
    %   s_dot = -Kr*s

    ua_r_SMC = -(Kr_SMC*s_SMC)/SB_SMC;
end

%% Boundary-layer saturation
if phi_SMC < 1e-6
    phi_use_SMC = 1e-6;
else
    phi_use_SMC = phi_SMC;
end

sat_arg_SMC = s_SMC/phi_use_SMC;

if sat_arg_SMC > 1
    sat_s_SMC = 1;
elseif sat_arg_SMC < -1
    sat_s_SMC = -1;
else
    sat_s_SMC = sat_arg_SMC;
end

%% Switching term
ua_sw_SMC = -Ksw_SMC*sat_s_SMC;

%% Total voltage command
ua_SMC = ua_eq_SMC + ua_r_SMC + ua_sw_SMC;

%% Voltage saturation
if ua_SMC > uMax_SMC
    ua_SMC = uMax_SMC;
elseif ua_SMC < -uMax_SMC
    ua_SMC = -uMax_SMC;
end