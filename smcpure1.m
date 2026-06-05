%% TwoState_SMC_Setup.m
% Two-state Sliding Mode Controller setup for self-balancing robot
%
% Sliding surface:
%   s = dot_theta + lambda_theta*theta
%
% Full state:
%   x = [gamma; theta; dot_gamma; dot_theta]
%
% Units:
%   [rad; rad; rad/s; rad/s]
%
% Controller output:
%   ua_SMC [V]
%
% Run this before Simulink.

Nominal_Tracking;

%% Sliding surface parameters
% s = dot_theta + lambda_theta*theta
%
% If s = 0, then:
%   dot_theta = -lambda_theta*theta
%
% Therefore theta converges approximately like:
%   theta(t) = theta(0)*exp(-lambda_theta*t)

lambda_theta_SMC = 8;

S_SMC = [0, lambda_theta_SMC, 0, 1];

%% Check S*B
SB_SMC = S_SMC*B;

disp('S_SMC = ');
disp(S_SMC);

disp('S_SMC*B = ');
disp(SB_SMC);

if abs(SB_SMC) < 1e-6
    error('Bad sliding surface: S_SMC*B is too close to zero. Change lambda_theta_SMC.');
end

%% SMC gains
% Reaching term:
%   u_r = -Kr*s/(S*B)
%
% Switching term:
%   u_sw = -Ksw*sat(s/phi)

Kr_SMC = 0.5;        % smooth reaching gain
Ksw_SMC = 1.0;       % switching gain [V]

%% Boundary layer
% Larger phi -> smoother, less chattering
% Smaller phi -> closer to ideal SMC, more aggressive

phi_SMC = 0.05;

%% Voltage saturation
uMax_SMC = drv.Vbus;     % usually 11.1 V

%% First test condition
dist_SMC = 0;

%% Suggested tuning
% Start with:
%   lambda_theta_SMC = 8;
%   Kr_SMC = 0.5;
%   Ksw_SMC = 1.0;
%   phi_SMC = 0.05;
%
% If balancing is weak:
%   Ksw_SMC = 1.5;
%   Kr_SMC = 1.0;
%
% If motor vibration/chattering is high:
%   phi_SMC = 0.08;
%   phi_SMC = 0.10;
%
% If tilt response is too slow:
%   lambda_theta_SMC = 10;
%
% If response is too aggressive:
%   lambda_theta_SMC = 5;