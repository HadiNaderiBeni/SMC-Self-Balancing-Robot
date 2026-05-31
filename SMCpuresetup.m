%% Pure_SMC_Regulator_Setup.m
% Pure periodic Sliding Mode Controller for balancing robot
% NO reference tracking
%
% State:
%   x = [gamma; theta; gamma_dot; theta_dot]
% units:
%   [rad; rad; rad/s; rad/s]
%
% Controller output:
%   ua_SMC [V]
%
% Run this before Simulink.

Nominal_Tracking;

%% Pure SMC sliding surface
% Sliding variable:
%
%   s = S_SMC*x
%
% We are NOT using tracking here.
% We are also NOT using Nx or Nu.
%
% Practical starting surface:
% gamma       weight: 1.0
% theta       weight: 25.0
% gamma_dot   weight: 0.8
% theta_dot   weight: 3.0

S_SMC = [1.0, 25.0, 0.8, 3.0];

%% Check S*B
SB_SMC = S_SMC*B;

if abs(SB_SMC) < 1e-6
    error('Bad sliding surface: S_SMC*B is too close to zero. Change S_SMC.');
end

%% SMC switching parameters
% Switching gain in volts.
% Start conservative on the real robot.

Ksw_SMC = 1.0;      % [V]

% Boundary layer thickness.
% Larger phi = smoother, less chattering.
% Smaller phi = stronger switching, more chattering.

phi_SMC = 0.05;

%% Reaching-rate gain
% Adds a smooth term that drives s toward zero.
% If this is too aggressive, reduce it.

Kr_SMC = 0.5;

%% Voltage saturation
uMax_SMC = drv.Vbus;        % normally 11.1 V

%% Disturbance kept zero for first test
dist_SMC = 0;

%% Recommended first tests
% If balancing is weak:
%   Ksw_SMC = 1.5;
%   Kr_SMC = 1.0;
%
% If chattering/vibration is high:
%   phi_SMC = 0.08;
%   phi_SMC = 0.10;