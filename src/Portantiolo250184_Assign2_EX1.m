% Modeling and Simulation of Aerospace Systems (2024/2025)
% Assignment # 2
% Author: Matteo Portantiolo

%% START
clearvars; close all; clc;  

% Setting plot options
set(groot, 'defaultTextInterpreter', 'latex')
set(groot, 'defaultAxesTickLabelInterpreter', 'latex')
set(groot, 'defaultLegendInterpreter','latex')
set(0, 'defaultAxesFontSize', 28 ,'defaultAxesFontSizeMode', 'manual');


%% Ex1 - SATELLITE THERMAL CONTROL

% Geometrical parameters of satellite components [m]
l1 = 1.5;
l2 = 0.5;
l3 = 0.5;
l_p = 0.95;
l_r = 0.5;

% Parameters of the thermal control mechanism (electromechanical)
R = 0.1;      % Electrical resistance [Ohm]
L = 0.001;    % Electrical inductance [H]
k_m = 0.3;    % Motor constant
m_r = 0.2;    % Moving mass [kg]

% Radiator configuration and physical limits
theta_min = -0.4*pi;  % Minimum allowed radiator angle [rad]
theta_max = 0;        % Maximum allowed radiator angle [rad]
eps_min = 0.01;       % Minimum emissivity
eps_max = 0.98;       % Maximum emissivity

% Thermal model parameters
T_ds = 3;               % Deep space temperature [K]
P_s = 1350;             % Solar power input [W/m^2]
C1 = 1.5e5;             % Heat capacity of body [J/K]
C2 = 1187.5;            % Heat capacity of panels [J/K]
C3 = 1187.5;            % Heat capacity of panels [J/K]
C4 = 30;                % Heat capacity of component 4 [J/K]
C5 = 30;                % Heat capacity of component 5 [J/K]
G12 = 10; G13 = 10; G14 = 10; G15 = 10;  % Thermal conductances [W/K]
alpha1 = 0.6; alpha2 = 0.78; alpha3 = 0.78;  % Absorptivities
eps1 = 0.45; eps2 = 0.75; eps3 = 0.75;      % Emissivities
sigma = 5.67e-8;        % Stefan–Boltzmann constant [W/m^2/K^4]

% Temperature thresholds and initial/reference values [K]
Tmax = 300;
Tmin = 290;
T0 = 298.15;                        % Initial temperature
Tref = 294.15;                      % Reference temperature
Tmax_osc = Tref + 0.001*Tref;       % Upper oscillation limit
Tmin_osc = Tref - 0.001*Tref;       % Lower oscillation limit

% Time settings
tf = 50*60*60;      % Final simulation time [s] (50 hours)
t_ref = 10*60*60;   % Reference time [s] (10 hours)


%% PROPAGATION WITH CONSTANT KP

% Proportional control gains (constant)
kp_min = 4.92e-5;   % Minimum value
kp_max = 1e-4;      % Maximum value (used in DYMOLA simulations)

% Initial state vector:
% [T1, T2, T3, T4, T5, theta, theta_dot, current]
x0 = [T0; T0; T0; T0; T0; theta_min; 0; 0];

% Set ODE solver options for stiff system
options = odeset('reltol', 1e-6, 'abstol', 1e-6);

% Propagate system using minimum kp
odefun = @(t,x) rhs(t,x,kp_min); 
[tt_min, xx_min] = ode15s(odefun, [0 tf], x0, options);

% Propagate system using maximum kp
odefun = @(t,x) rhs(t,x,kp_max); 
[tt_max, xx_max] = ode15s(odefun, [0 tf], x0, options);

% Extract selected states for kp_min case
T1_min = xx_min(:,1);            % Body temperature
T2_min = xx_min(:,2);            % Panel temperature
T4_min = xx_min(:,4);            % Radiator temperature
theta_minkp = xx_min(:,6);       % Radiator angle
theta_dot_min = xx_min(:,7);     % Angular velocity
i_min = xx_min(:,8);             % Electric current

% Extract selected states for kp_max case
T1_max = xx_max(:,1);
T2_max = xx_max(:,2);
T4_max = xx_max(:,4);
theta_maxkp = xx_max(:,6);
theta_dot_max = xx_max(:,7);
i_max = xx_max(:,8);

% Plot results

% Plot body temperature for both kp values
figure
plot(tt_min./3600, T1_min, 'Color', "#0072BD", 'LineWidth', 5)
hold on
plot(tt_max./3600, T1_max, 'Color', "#A2142F", 'LineWidth', 5)
yline(Tref, '--', 'LineWidth', 2)
yline(Tmin_osc, '--', 'Color', 'r', 'LineWidth', 2)
yline(Tmax_osc, '--', 'Color', 'r', 'LineWidth', 2)
xline(t_ref/3600, '--', 'Color', 'g', 'LineWidth', 2)
grid on
xlabel('t [h]')
ylabel('$T_{body}$ [K]')
ylim([292 299])
legend({'$T_{body}$ with $k_p$ min', ...
        '$T_{body}$ with $k_p$ max', ...
        '$T_{ref}$','$T_{limit}$'}, FontSize = 33)

% Plot panel and radiator temperatures
figure
subplot(1,2,1)
plot(tt_min./3600, T2_min, 'Color', "#0072BD", 'LineWidth', 5)
hold on
plot(tt_max./3600, T2_max, 'Color', "#A2142F", 'LineWidth', 5)
grid on
xlabel('t [h]')
ylabel('$T_{panels}$ [K]')
title('$T_{panels}$ evolution with constant $k_p$', 'Interpreter', 'Latex', FontSize = 33)
legend({'$T_{panels}$ with $k_p$ min','$T_{panels}$ with $k_p$ max'}, FontSize = 33)

subplot(1,2,2)
plot(tt_min./3600, T4_min, 'Color', "#0072BD", 'LineWidth', 5)
hold on
plot(tt_max./3600, T4_max, 'Color', "#A2142F", 'LineWidth', 5)
grid on
xlabel('t [h]')
ylabel('$T_{radiators}$ [K]')
title('$T_{radiators}$ evolution with constant $k_p$', 'Interpreter', 'Latex', FontSize = 33)
legend({'$T_{radiators}$ with $k_p$ min','$T_{radiators}$ with $k_p$ max'}, FontSize = 33)

% Plot radiator angle and angular velocity
figure
subplot(2,1,1)
plot(tt_min./3600, rad2deg(theta_minkp), 'Color', "#0072BD", 'LineWidth', 5)
hold on
plot(tt_max./3600, rad2deg(theta_maxkp), 'Color', "#A2142F", 'LineWidth', 5)
yline(rad2deg(theta_min), '--', 'Color', 'r', 'LineWidth', 2)
yline(rad2deg(theta_max), '--', 'Color', 'r', 'LineWidth', 2)
grid on
xlabel('t [h]')
ylabel('$\theta$ [deg]')
ylim([-75 1])
title('$\theta$ evolution with constant $k_p$', 'Interpreter', 'Latex', FontSize = 33)
legend({'$\theta$ with $k_p$ min','$\theta$ with $k_p$ max','$\theta_{limit}$'}, FontSize = 33)

subplot(2,1,2)
plot(tt_min./3600, rad2deg(theta_dot_min), 'Color', "#0072BD", 'LineWidth', 5)
hold on
plot(tt_max./3600, rad2deg(theta_dot_max), 'Color', "#A2142F", 'LineWidth', 5)
grid on
xlabel('t [h]')
ylabel('$\dot{\theta}$ [deg/s]')
title('$\dot{\theta}$ evolution with constant $k_p$', 'Interpreter', 'Latex', FontSize = 33)
legend({'$\dot{\theta}$ with $k_p$ min','$\dot{\theta}$ with $k_p$ max'}, FontSize = 33)

% Optional: Plot of current (commented out)
% figure
% plot(tt_min./3600, i_min, 'LineWidth', 2)
% hold on
% plot(tt_max./3600, i_max, 'LineWidth', 2)
% grid on
% xlabel('t')
% ylabel('i')
% title('Current evolution with constant $k_p$', 'Interpreter', 'Latex')
% legend(["i with $k_p$ min","i with $k_p$ max"], 'Location', 'best', FontSize=14)


%% PROPAGATION WITH ADAPTIVE KP 

% This section propagates the thermal control system using an adaptive
% proportional gain (kp) and plots the results. The gain (kp) starts from
% a minimum value and doubles each time until it reaches a specified maximum.

% Initial value of kp and the maximum allowed value
kp0 = 1e-5;   % Initial proportional gain value
kp_max = 1e-4; % Maximum allowed proportional gain

% Initial state vector [T1, T2, T3, T4, T5, theta, theta_dot, i]
x0 = [T0; T0; T0; T0; T0; theta_min; 0; 0]; 

% Call the 'propagate' function to solve the system with adaptive kp
[tt, xx, kp_vect] = propagate(x0, tf, kp0, kp_max);

% Extract specific variables from the solution
T1 = xx(:,1);      % Body temperature
theta = xx(:,6);   % Angle (theta)
theta_dot = xx(:,7); % Angular velocity (theta_dot)
i = xx(:,8);       % Current (i)

% Plotting the body temperature evolution over time
figure
plot(tt./3600, T1, 'Color', "#0072BD", 'LineWidth', 5)  % Body temperature plot
hold on
yline(Tref, '--', 'LineWidth', 2)  % Reference temperature line
yline(Tmin_osc, '--', 'Color', 'r', 'LineWidth', 2)  % Minimum temperature oscillation limit
yline(Tmax_osc, '--', 'Color', 'r', 'LineWidth', 2)  % Maximum temperature oscillation limit
xline(t_ref/3600, '--', 'Color', 'g', 'LineWidth', 2)  % Reference time line
grid on
xlabel('t [h]')  % X-axis label (time in hours)
ylabel('$T_{body}$ [K]')  % Y-axis label (body temperature in Kelvin)
ylim([292 299])  % Set Y-axis limits for body temperature
%title('$T_1$ evolution with adaptive $k_p$', 'Interpreter', 'Latex', 'FontSize', 33)
legend({'$T_{body}$','$T_{ref}$','$T_{limit}$'}, 'FontSize', 33)  % Legend for the plot

% Plotting the evolution of theta (angle) with adaptive kp
figure
subplot(2, 1, 1)
plot(tt./3600, rad2deg(theta), 'Color', "#0072BD", 'LineWidth', 5)  % Angle plot
hold on
yline(rad2deg(theta_min), '--', 'Color', 'r', 'LineWidth', 2)  % Minimum angle limit
yline(rad2deg(theta_max), '--', 'Color', 'r', 'LineWidth', 2)  % Maximum angle limit
grid on
xlabel('t [h]')  % X-axis label (time in hours)
ylabel('$\theta$ [deg]')  % Y-axis label (angle in degrees)
ylim([-75 1]);  % Set Y-axis limits for angle
title('$\theta$ evolution with adaptive $k_p$', 'Interpreter', 'Latex', 'FontSize', 33)
legend({"$\theta$","$\theta_{limit}$"}, 'FontSize', 33)  % Legend for the plot

% Plotting the evolution of theta_dot (angular velocity) with adaptive kp
subplot(2, 1, 2)
plot(tt./3600, rad2deg(theta_dot), 'Color', "#0072BD", 'LineWidth', 5)  % Angular velocity plot
grid on
xlabel('t [h]')  % X-axis label (time in hours)
ylabel('$\dot{\theta}$ [deg/s]')  % Y-axis label (angular velocity in degrees per second)
title('$\dot{\theta}$ evolution with adaptive $k_p$', 'Interpreter', 'Latex', 'FontSize', 33)  % Title for angular velocity plot


%% DYMOLA

% This section loads the data from a Dymola simulation and compares it to the 
% results obtained from the Matlab simulation. Several variables are plotted
% to visualize the comparison between the two methods.

% === FILE LOADING ===
% Load the .mat file containing the Dymola simulation results
data = load('Portantiolo250184_Assign2_EX_1_res.mat');

% Create a figure to display the comparisons between Dymola and Matlab results
figure;

% --- Comparison for Body Temperature (T_body) ---
subplot(3, 2, 1);
% Plot Dymola data for body temperature vs time
plot(data.data_2(1, :) / 3600, data.data_2(5, :), 'LineWidth', 5, 'LineStyle', '--');
hold on;
% Plot Matlab results for body temperature vs time
plot(tt_max ./ 3600, T1_max, 'LineWidth', 3);
grid on;
legend('Dymola', 'Matlab', 'FontSize', 33);
xlabel('t [h]');
ylabel('$T_{body}$ [K]');
title('$T_{body}$ comparison');

% --- Comparison for Panels Temperature (T_panles) ---
subplot(3, 2, 2);
% Plot Dymola data for panels temperature vs time
plot(data.data_2(1, :) / 3600, data.data_2(6, :), 'LineWidth', 5, 'LineStyle', '--');
hold on;
% Plot Matlab results for panels temperature vs time
plot(tt_max ./ 3600, T2_max, 'LineWidth', 3);
grid on;
xlabel('t [h]');
ylabel('$T_{panles}$ [K]');
title('$T_{panles}$ comparison');

% --- Comparison for Radiators Temperature (T_radiators) ---
subplot(3, 2, 3);
% Plot Dymola data for radiators temperature vs time
plot(data.data_2(1, :) / 3600, data.data_2(8, :), 'LineWidth', 5, 'LineStyle', '--');
hold on;
% Plot Matlab results for radiators temperature vs time
plot(tt_max ./ 3600, T4_max, 'LineWidth', 3);
grid on;
xlabel('t [h]');
ylabel('$T_{radiators}$ [K]');
title('$T_{radiators}$ comparison');

% --- Comparison for Current (I) ---
subplot(3, 2, 4);
% Plot Dymola data for current vs time
plot(data.data_2(1, :) / 3600, data.data_2(2, :), 'LineWidth', 5, 'LineStyle', '--');
hold on;
% Plot Matlab results for current vs time
plot(tt_max ./ 3600, i_max, 'LineWidth', 3);
grid on;
xlabel('t [h]');
ylabel('$I$ [A]');
title('$I$ comparison');
ylim([-4e-8, 4e-8]); % Set Y-axis limits for current comparison

% --- Comparison for Angle (theta) ---
subplot(3, 2, 5);
% Plot Dymola data for angle (theta) vs time
plot(data.data_2(1, :) / 3600, rad2deg(data.data_2(3, :)), 'LineWidth', 5, 'LineStyle', '--');
hold on;
% Plot Matlab results for angle (theta) vs time
plot(tt_max ./ 3600, rad2deg(theta_maxkp), 'LineWidth', 3);
grid on;
xlabel('t [h]');
ylabel('$\theta$ [rad]');
title('$\theta$ comparison');

% --- Comparison for Angular Velocity (theta_dot) ---
subplot(3, 2, 6);
% Plot Dymola data for angular velocity (theta_dot) vs time
plot(data.data_2(1, :) / 3600, rad2deg(data.data_2(4, :)), 'LineWidth', 5, 'LineStyle', '--');
hold on;
% Plot Matlab results for angular velocity (theta_dot) vs time
plot(tt_max ./ 3600, rad2deg(theta_dot_max), 'LineWidth', 3);
grid on;
xlabel('t [h]');
ylabel('$\dot\theta$ [rad/s]');
title('$\dot\theta$ comparison');


%% FUNCTIONS


function K = const()
%--------------------------------------------------------------------------
% Author: Matteo Portantiolo
%
% CONST Defines and returns a structure containing constant parameters.
%
% This function groups and returns all the physical, thermal, geometric,
% and simulation constants used in the thermal control model of a satellite.
% It helps centralize the parameter definitions for better readability and maintainability.
%
% OUTPUT:
%   K : structure containing all model constants
%--------------------------------------------------------------------------


    % ------------------------------
    % Geometric parameters [m]
    % ------------------------------
    K.l1 = 1.5;     % length of body 1
    K.l2 = 0.5;     % length of body 2 (e.g., panel)
    K.l3 = 0.5;     % length of body 3 (e.g., panel)
    K.l_p = 0.95;   % length parameter for the panel mechanism
    K.l_r = 0.5;    % length parameter for the radiator

    % ------------------------------
    % Thermal control mechanism data
    % ------------------------------
    K.R = 0.1;      % electrical resistance [Ohm]
    K.L = 0.001;    % inductance [H]
    K.k_m = 0.3;    % motor constant [Nm/A]
    K.m_r = 0.2;    % moment of inertia [kg·m²]

    % ------------------------------
    % Radiator constraints
    % ------------------------------
    K.theta_min = -0.4*pi;  % minimum angle for radiator opening [rad]
    K.theta_max = 0;        % maximum angle for radiator opening [rad]
    K.eps_min = 0.01;       % minimum emissivity
    K.eps_max = 0.98;       % maximum emissivity

    % ------------------------------
    % Thermal parameters
    % ------------------------------
    K.T_ds = 3;             % deep space temperature [K]
    K.P_s = 1350;           % solar power density [W/m²]
    K.C1 = 1.5e5;           % thermal capacity of body 1 [J/K]
    K.C2 = 1187.5;          % thermal capacity of body 2 [J/K]
    K.C3 = 1187.5;          % thermal capacity of body 3 [J/K]
    K.C4 = 30;              % thermal capacity of mechanism 1 [J/K]
    K.C5 = 30;              % thermal capacity of mechanism 2 [J/K]

    % Thermal conductances [W/K]
    K.G12 = 10;
    K.G13 = 10;
    K.G14 = 10;
    K.G15 = 10;

    % Absorptivity coefficients
    K.alpha1 = 0.6;
    K.alpha2 = 0.78;
    K.alpha3 = 0.78;

    % Emissivity coefficients
    K.eps1 = 0.45;
    K.eps2 = 0.75;
    K.eps3 = 0.75;

    % Stefan-Boltzmann constant [W/(m²·K⁴)]
    K.sigma = 5.67e-8;

    % ------------------------------
    % Temperature thresholds [K]
    % ------------------------------
    K.Tmax = 300;                            % maximum allowed temperature
    K.Tmin = 290;                            % minimum allowed temperature
    K.T0 = 298.15;                           % initial temperature
    K.Tref = 294.15;                         % reference temperature
    K.Tmax_osc = K.Tref + 0.001 * K.Tref;    % maximum oscillation threshold
    K.Tmin_osc = K.Tref - 0.001 * K.Tref;    % minimum oscillation threshold

    % ------------------------------
    % Simulation time settings [s]
    % ------------------------------
    K.tf = 50 * 60 * 60;     % total simulation time (50 hours)
    K.t_ref = 10 * 60 * 60;  % reference time (10 hours)

end


function dxdt = rhs(~, x, k_p)
%--------------------------------------------------------------------------
% Author: Matteo Portantiolo
%
% RHS Computes the right-hand side of the ODE system for satellite thermal control.
%
% This function implements the system dynamics of a satellite thermal control
% problem, including the thermal behavior of five subsystems and the dynamics
% of a deployable radiator mechanism with electromechanical actuation.
%
% INPUTS:
%   x   : state vector [T1; T2; T3; T4; T5; theta; theta_dot; i]
%         where:
%           T1 - temperature of main body [K]
%           T2 - temperature of panel 1 [K]
%           T3 - temperature of panel 2 [K]
%           T4 - temperature of radiator 1 [K]
%           T5 - temperature of radiator 2 [K]
%           theta - radiator rotation angle [rad]
%           theta_dot - angular velocity of the radiator [rad/s]
%           i - current in the actuator circuit [A]
%   k_p : proportional control gain [V/K]
%
% OUTPUT:
%   dxdt : time derivative of the state vector
%--------------------------------------------------------------------------

    dxdt = NaN(length(x),1); % Preallocate output vector

    K = const(); % Load all constants

    % Extract states
    T1 = x(1);         % Main body temperature
    T2 = x(2);         % Panel 1 temperature
    T3 = x(3);         % Panel 2 temperature
    T4 = x(4);         % Radiator 1 temperature
    T5 = x(5);         % Radiator 2 temperature
    theta = x(6);      % Radiator angle
    theta_dot = x(7);  % Radiator angular velocity
    i = x(8);          % Circuit current

    % -----------------------------
    % Control logic (saturation)
    % -----------------------------
    if theta < (K.theta_min + 0.001 * K.theta_min) && T1 < K.Tref
        k_p = 0; % Deactivate actuator when at lower bound and temperature is low
    end
    if theta > (K.theta_max - 0.001 * K.theta_max) && T1 > K.Tref
        k_p = 0; % Deactivate actuator when at upper bound and temperature is high
    end

    % -----------------------------
    % Auxiliary parameters
    % -----------------------------
    A_tot = 2*K.l2*K.l3 + 4*K.l1*K.l2; % Total radiating area
    eps4 = K.eps_min + (K.eps_max - K.eps_min)/(0.4*pi) * (theta + 0.4*pi); % emissivity radiator 1
    eps5 = K.eps_min + (K.eps_max - K.eps_min)/(0.4*pi) * (theta + 0.4*pi); % emissivity radiator 2
    tau_in = K.k_m * i;                   % torque input from motor
    J_r = (1/3) * K.m_r * K.l_r^2;        % moment of inertia of the radiator
    V_in = k_p * (T1 - K.Tref);           % control voltage input

    % -----------------------------
    % System dynamics
    % -----------------------------
    % T1: main body energy balance
    dxdt(1) = ( K.alpha1*K.P_s*K.l2*K.l3 ...                          % solar gain
              - K.eps1*A_tot*K.sigma*(T1^4 - K.T_ds^4) ...           % radiation loss
              + K.G12*(T2 - T1) + K.G13*(T3 - T1) ...                % conduction to panels
              + K.G14*(T4 - T1) + K.G15*(T5 - T1) ) / K.C1;          % conduction to radiators

    % T2: panel 1 energy balance
    dxdt(2) = ( K.alpha2*K.P_s*K.l2*K.l_p ...
              - K.eps2*K.l2*K.l_p*K.sigma*(T2^4 - K.T_ds^4) ...
              - K.G12*(T2 - T1) ) / K.C2;

    % T3: panel 2 energy balance
    dxdt(3) = ( K.alpha3*K.P_s*K.l3*K.l_p ...
              - K.eps3*K.l3*K.l_p*K.sigma*(T3^4 - K.T_ds^4) ...
              - K.G13*(T3 - T1) ) / K.C3;

    % T4: radiator 1 energy balance
    dxdt(4) = ( -eps4*K.l_r*K.l2*K.sigma*(T4^4 - K.T_ds^4) ...
              - K.G14*(T4 - T1) ) / K.C4;

    % T5: radiator 2 energy balance
    dxdt(5) = ( -eps5*K.l_r*K.l2*K.sigma*(T5^4 - K.T_ds^4) ...
              - K.G15*(T5 - T1) ) / K.C5;

    % theta: angular position
    dxdt(6) = theta_dot;

    % theta_dot: angular acceleration
    dxdt(7) = tau_in / J_r;

    % i: actuator current dynamics (RL circuit with back-emf)
    dxdt(8) = V_in/K.L - K.R/K.L*i - K.k_m/K.L*theta_dot;

end


function [value, isterminal, direction] = adaptive_kp(~, y, isTerminal)
%--------------------------------------------------------------------------
% Author: Matteo Portantiolo
%
% ADAPTIVE_KP Event function for detecting when the main body temperature (T1) crosses the reference.
%
% This function is typically used in an ODE solver (like ode45) to detect when
% the temperature T1 crosses the reference value `Tref`, in order to trigger 
% control logic such as changing the proportional gain `k_p`.
%
% INPUTS:
%   ~           : Time (unused, included for compatibility with ODE solvers)
%   y           : State vector (must include T1 as first element)
%   isTerminal  : Boolean flag; if true, the ODE solver stops when the event occurs
%
% OUTPUTS:
%   value       : Event condition (zero when T1 == Tref)
%   isterminal  : Whether the event should stop the integration (true/false)
%   direction   : Direction of zero-crossing to detect (0 = any direction)
%--------------------------------------------------------------------------

    K = const();         % Load physical constants
    T1 = y(1);           % Extract temperature of main body
    value = T1 - K.Tref; % Event condition: difference from reference temperature
    isterminal = isTerminal; % Whether to stop integration at event
    direction = 0;       % Detect zero-crossing in any direction

end


function [tt, xx, kp_vect] = propagate(x0, tf, kp, kp_max)
%--------------------------------------------------------------------------
% Author: Matteo Portantiolo
%
% PROPAGATE Solves the system of differential equations with an adaptive
%           proportional gain that increases over time. The ODE solver 
%           integrates the system until a stopping event occurs or the 
%           specified final time is reached. The proportional gain (kp) 
%           is doubled each time the event triggers, up to a maximum value.
%
% INPUTS:
%   x0       - Initial state vector for the ODE system
%   tf       - Final time for the simulation
%   kp       - Initial proportional gain for the control system
%   kp_max   - Maximum value for the proportional gain (adaptive control)
%
% OUTPUTS:
%   tt       - Time vector of the ODE solution
%   xx       - Solution matrix (states at each time step)
%   kp_vect  - Vector tracking the proportional gain at each iteration
%--------------------------------------------------------------------------

    % Initialize the proportional gain vector
    kp_vect(1) = kp; 
    
    % Initialize time and state arrays
    tt = [];
    xx = [];
    
    % Starting time for integration
    t_in = 0;
    
    % Iteration counter
    iter = 1;
    
    % Loop until the final time is reached
    while t_in < tf
        
        % Set ODE solver options (with a small tolerance)
        options = odeset('reltol', 1e-6, 'abstol', 1e-6);
        
        % If kp is less than the maximum, add an event function for adaptive control
        if kp < kp_max
            options = odeset('reltol', 1e-6, 'abstol', 1e-6, 'Events', @(t, y) adaptive_kp(t, y, true));
        end
        
        % Define the right-hand side of the system of ODEs
        odefun = @(t, x) rhs(t, x, kp);
        
        % Solve the system of ODEs using ode15s (for stiff systems)
        [tt_prev, xx_prev] = ode15s(odefun, [t_in tf], x0, options);
        
        % If kp is less than kp_max, increase kp and store the new value in the vector
        if kp < kp_max
            kp = 2 * kp;
            kp_vect(iter + 1) = kp;
        end
        
        % Update the current time and state
        t_in = tt_prev(end);
        tt = [tt; tt_prev];    % Append new time values
        x0 = xx_prev(end, :);  % Set the new initial condition for the next iteration
        xx = [xx; xx_prev];    % Append new state values
        
        % Increment the iteration counter
        iter = iter + 1;
    end
end

