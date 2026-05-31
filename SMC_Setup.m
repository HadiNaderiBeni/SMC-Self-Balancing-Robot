%% SMC_Setup.m
% Practical sliding mode controller setup for balancing robot
% Run after Nominal_Tracking.m

Nominal_Tracking;

%% Use the LQR gain direction as sliding surface
% State:
% x = [gamma; theta; gamma_dot; theta_dot]
% units = [rad; rad; rad/s; rad/s]
%
% Sliding variable:
% s = S_SMC * x

S_SMC = K_Nom_LQR_Br;

%% Check that S*B is nonzero
SB_SMC = S_SMC*B;

if abs(SB_SMC) < 1e-6
    error('Bad sliding surface: S_SMC*B is too close to zero.');
end

%% Switching gain [V]
% Start small. Increase only if needed.
Ksw_SMC = 1.5;

%% Boundary layer thickness
% Larger phi = smoother, less chattering, weaker robustness.
% Smaller phi = more aggressive, more chattering.
phi_SMC = 0.05;

%% Voltage saturation
uMax_SMC = drv.Vbus;      % 11.1 V

%% Optional low-pass / rate limiting can be added later

%start
%Ksw_SMC = 1.0;
%phi_SMC = 0.05;

%then
%Ksw_SMC = 1.5;
%phi_SMC = 0.05;

%if it vibrates too much
% phi_SMC = 0.10;

%if it doesnt recover strongly enough
% Ksw_SMC = 2.0;