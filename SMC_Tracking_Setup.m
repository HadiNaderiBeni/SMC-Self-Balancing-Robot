%% SMC_Tracking_Setup.m
% Practical sliding mode controller for CONSTANT reference tracking
% Balancing robot state:
%   x = [gamma; theta; gamma_dot; theta_dot]
% units:
%   [rad; rad; rad/s; rad/s]
%
% Run this before Simulink.

Nominal_Tracking;

%% Tracking feedforward terms
% For constant reference r = gam_ref_rad:
%
%   x_ss = Nx_SMC*r
%   ua_ss = Nu_SMC*r
%
% Tracking error:
%
%   x_tilde = x - Nx_SMC*r

Nx_SMC = Nx;       % 4x1
Nu_SMC = Nu;       % scalar

%% Sliding surface
% Use the nominal LQR gain as a practical stabilizing sliding surface.
% Sliding variable:
%
%   s = S_SMC*x_tilde

S_SMC = K_Nom_LQR_Br;     % 1x4

%% Check S*B
SB_SMC = S_SMC*B;

if abs(SB_SMC) < 1e-6
    error('Bad sliding surface: S_SMC*B is too close to zero.');
end

%% SMC tuning parameters
% Switching gain in volts.
% Start small. Increase only if the robot is weak against disturbance.
Ksw_SMC = 1.0;

% Boundary layer thickness.
% Larger phi = smoother, less chattering.
% Smaller phi = stronger switching, more chattering.
phi_SMC = 0.05;

%% Voltage saturation
uMax_SMC = drv.Vbus;      % normally 11.1 V

%% Constant reference
% Use radians.
% Start with zero, then try small references.

gam_ref_SMC = 0;
% gam_ref_SMC = 2*deg2rad;
% gam_ref_SMC = 5*deg2rad;

%% Disturbance kept zero for first test
dist_SMC = 0;