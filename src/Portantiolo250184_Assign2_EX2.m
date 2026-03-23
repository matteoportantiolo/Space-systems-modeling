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


%% Ex2 SATELLITE DRAG COMPENSATION

% Accelerometer parameters
m_sc = 300;  % Spacecraft mass [kg]
m_a = 0.32;  % Accelerometer mass [kg]
ba_min = 1.5e3;  % Minimum damping coefficient for accelerometer [Ns/m]
ba_max = 2e4;  % Maximum damping coefficient for accelerometer [Ns/m]
ka_min = 5e-5;  % Minimum spring constant for accelerometer [N/m]
ka_max = 3e-3;  % Maximum spring constant for accelerometer [N/m]
k_acc = 1;  % Gain for accelerometer

% Amplifier parameters
Rin_min = 0.1;  % Minimum input resistance for amplifier [ohm]
Rin_max = 10;  % Maximum input resistance for amplifier [ohm]
Rf_min = 1e4;  % Minimum feedback resistance for amplifier [ohm]
Rf_max = 8e4;  % Maximum feedback resistance for amplifier [ohm]

% Solenoid Valve parameters
m_v = 0.1;  % Solenoid valve mass [kg]
k_v = 1e3;  % Solenoid valve spring constant [N/m]
b_v = 1e3;  % Solenoid valve damping coefficient [Ns/m]
alpha = 2.1e-2;  % Solenoid valve parameter
beta = -60;  % Solenoid valve parameter
A0 = 4.7e-12;  % Solenoid valve area constant [m^2]
l = 1e-5;  % Solenoid valve length [m]

% Thruster parameters
k = 1.66;  % Specific heat ratio
p_t = 2e5;  % Xenon gas pressure [Pa]
T_t = 240;  % Xenon temperature [K]
R = 63.32754;  % Specific gas constant for Xenon [J/kg/K]
q = 1.6e-19;  % Charge of an electron [C]
dV = 2000;  % Voltage [V]
m_i = 2.188e-25;  % Ion mass [kg]

% Drag parameters
omega_s = 1.658226e-6;  % Satellite angular frequency [rad/s]
omega0 = 1.160758e-3;  % Orbital frequency [rad/s]

% Data tuning
ba = 1.5e3;  % Damping coefficient for accelerometer [Ns/m]
ka = 3e-3;   % Spring constant for accelerometer [N/m]
Rin = 0.1;   % Input resistance for amplifier [ohm]
Rf = 8e4;    % Feedback resistance for amplifier [ohm]
var = [ba ka Rin Rf];  % Parameter vector for system

% Time parameters
t0 = 0;    % Initial time [s]
T_orbit = 2*pi/omega0;  % Orbital period [s]
tf = 3*T_orbit;   % Final time [s] (three orbital periods)

% Xenon mass calculation (worst-case scenario)
m_flow =(A0+l^2) * sqrt(k * (p_t)/(R*T_t) * p_t * (2/(k+1))^((k+1)/(k-1)));  % Xenon mass flow rate
m_Xe = m_flow * tf;  % Total mass of Xenon for three orbital periods


%% GENERAL INITIAL CONDITIONS

% Initial conditions for the system (spacecraft and actuator positions)
x0 = [0; 0; l; 0; 0];  

% Perform integration using ode15s solver with relative and absolute tolerance
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[tt,xx] = ode15s(@(t,x) rhs(t,x,var), [t0 tf], x0, options);

% Compute Drag force as a function of time
D = (2.2 - cos(omega_s.*tt) + 1.2.*sin(omega0.*tt).*cos(omega0.*tt)) * 1e-3;

% Compute Thrust force as a function of time
Av = A0 + l*(l-xx(:,3));  % Area of the solenoid valve
rho = p_t / (R*T_t);  % Xenon density
m_dot = Av * sqrt( k*rho*p_t * (2/(k+1))^((k+1)/(k-1)) );  % Mass flow rate
T = m_dot * sqrt(2*q*dV/m_i);  % Thrust force

% Plots for thrust, drag, and errors

figure
subplot(1,2,1)
plot(tt/T_orbit,T,'Color',"#0072BD",'LineWidth',5)  % Plot thrust
hold on 
plot(tt/T_orbit,D,'Color',"#A2142F",'LineWidth',5,'LineStyle','--')  % Plot drag
legend('Thrust','Drag','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('T,D [N]')
title ('Thrust vs Drag','Interpreter','Latex',FontSize = 33)
grid on

subplot(1,2,2)
semilogy(tt/T_orbit, abs(T - D), 'LineWidth', 5);  % Plot absolute error
xlabel('t [orbital periods]')
ylabel('$\left| T - D \right| \, [N]$')
title('Absolute error')
grid on

% Additional plots for spacecraft and actuator states

figure
subplot(2,2,1)
plot(tt/T_orbit, xx(:,1),'Color',"#0072BD", 'LineWidth', 4)  % Seismic mass position
xlabel('t [orbital periods]')
ylabel('$x_a$ [m]') 
grid on
ylim([1.7e-8,2.5e-8])
title('Seismic mass position','Interpreter','Latex',FontSize = 33)

subplot(2,2,2)
plot(tt/T_orbit, xx(:,2),'Color',"#0072BD", 'LineWidth', 4)  % Seismic mass velocity
xlabel('t [orbital periods]')
ylabel('$v_a$ [m/s]')
grid on
ylim([-6e-12,6e-12])
title('Seismic mass velocity','Interpreter','Latex',FontSize = 33)

subplot(2,2,3)
plot(tt/T_orbit, xx(:,3),'Color',"#A2142F", 'LineWidth', 4)  % Spool position
hold on
yline(1e-5,'LineWidth',3,'LineStyle','--')  % Maximum spool position
legend('$x_v$','$x_{v,max}$','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('$x_v$ [m]') 
grid on
ylim([5e-6,12.5e-6])
title('Spool position','Interpreter','Latex',FontSize = 33)

subplot(2,2,4)
hold on
plot(tt/T_orbit, xx(:,4),'Color',"#A2142F", 'LineWidth', 4)  % Spool velocity
xlabel('t [orbital periods]')
ylabel('$v_v$ [m/s]') 
grid on
ylim([-5.5e-9,5.5e-9])
title('Spool velocity','Interpreter','Latex',FontSize = 33)

figure
plot(tt/T_orbit, xx(:,5), 'LineWidth', 4, 'Color', [0.3, 0.7, 0.3])  % Current
xlabel('t [orbital periods]')
ylabel('$I$ [A]') 
grid on
title('Current','Interpreter','Latex',FontSize = 33)

% Transient phase plots

figure
subplot(1,2,1)
plot(tt,T,'Color',"#0072BD",'LineWidth',5)  % Plot thrust in time domain
hold on 
plot(tt,D,'Color',"#A2142F",'LineWidth',5,'LineStyle','--')  % Plot drag in time domain
legend('Thrust','Drag','Interpreter','Latex',FontSize = 33)
xlabel('t [s]')
ylabel('T,D [N]')
title ('Thrust vs Drag','Interpreter','Latex',FontSize = 33)
grid on
xlim([0,30])

subplot(1,2,2)
semilogy(tt, abs(T - D), 'LineWidth', 5);  % Plot absolute error in time domain
xlabel('t [s]')
ylabel('$\left| T - D \right| \, [N]$')
title('Absolute error','Interpreter','Latex',FontSize = 33)
grid on
xlim([0,30])

figure
subplot(2,2,1)
plot(tt, xx(:,1),'Color',"#0072BD", 'LineWidth', 4)  % Seismic mass position over time
xlabel('t [s]')
ylabel('$x_a$ [m]') 
grid on
title('Seismic mass position','Interpreter','Latex',FontSize = 33)
xlim([0,30])
ylim([-0.2e-8,2.2e-8])

subplot(2,2,2)
plot(tt, xx(:,2),'Color',"#0072BD", 'LineWidth', 4)  % Seismic mass velocity over time
xlabel('t [s]')
ylabel('$v_a$ [m/s]')
grid on
title('Seismic mass velocity','Interpreter','Latex',FontSize = 33)
xlim([0,30])
ylim([-1.5e-9,4.5e-9])

subplot(2,2,3)
plot(tt, xx(:,3),'Color',"#A2142F", 'LineWidth', 4)  % Spool position over time
xlabel('t [s]')
ylabel('$x_v$ [m]') 
grid on
title('Spool position','Interpreter','Latex',FontSize = 33)
xlim([0,30])

subplot(2,2,4)
hold on
plot(tt, xx(:,4),'Color',"#A2142F", 'LineWidth', 4)  % Spool velocity over time
xlabel('t [s]')
ylabel('$v_v$ [m/s]') 
grid on
title('Spool velocity','Interpreter','Latex',FontSize = 33)
xlim([0,30])
ylim([-13e-6,2.5e-6])

figure
plot(tt, xx(:,5), 'LineWidth', 4, 'Color', [0.3, 0.7, 0.3])  % Current over time
xlabel('t [s]')
ylabel('$I$ [A]') 
grid on
title('Current','Interpreter','Latex',FontSize = 33)
xlim([0,30])


%% OPTIMAL INITIAL CONDITIONS

% Initialize parameters for optimization loop
Nmax = 1000; % Maximum iterations for the optimization loop
iter = 1; % Initial iteration count
threshold = 1e-6; % Convergence threshold for error
current_error = 1; % Initialize current error as a large value
previous_error = 0; % Initialize previous error as 0
x0_opt = x0; % Set the initial optimal initial conditions to the starting values

% Start optimization loop to find the optimal initial conditions
while iter < Nmax && abs(current_error - previous_error) > threshold
    % Set ODE solver options with high tolerance for precision
    options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);

    % Solve ODE with the current guess for initial conditions
    [time, state] = ode15s(@(t, x) rhs(t,x,var), [t0, tf], x0_opt, options);

    % Calculate drag force based on the given formula (time-dependent)
    D = (2.2 - cos(omega_s .* time) + 1.2 .* sin(omega0 .* time) .* cos(omega0 .* time)) * 1e-3;

    % Calculate tank density for the ion thruster
    tank_density = p_t / (R * T_t);

    % Calculate valve area, which depends on the spool position (state(:, 3))
    valve_area = A0 + l * (l - state(:, 3));

    % Calculate mass flow rate through the ion thruster valve
    mass_flow_rate = valve_area .* sqrt(k * tank_density * p_t * (2 / (k + 1))^((k + 1) / (k - 1)));

    % Calculate exhaust velocity of the ion thruster
    exhaust_velocity = sqrt(2 * q * dV / m_i);

    % Calculate thrust generated by the ion thruster
    T = mass_flow_rate * exhaust_velocity;

    % Update the error between thrust and drag
    previous_error = current_error;
    current_error = abs(T(1) - D(1)); % Compare the first values of thrust and drag

    % Check if the error is below the threshold, indicating convergence
    if current_error < threshold
        break;
    end

    % Update the initial conditions for the next iteration
    iter = iter + 1;
    x0_opt = state(end, :); % Set the new initial conditions as the last state
end

% Perform integration using the optimized initial conditions
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[tt,xx] = ode15s(@(t,x) rhs(t,x,var), [t0 tf], x0_opt, options);

% Compute drag force using the time-dependent formula
D = (2.2 - cos(omega_s.*tt) + 1.2.*sin(omega0.*tt).*cos(omega0.*tt)) * 1e-3;

% Compute thrust for the ion thruster
Av = A0 + l*(l-xx(:,3)); % Area of the solenoid valve
rho = p_t / (R*T_t); % Density of the gas in the tank
m_dot = Av * sqrt(k * rho * p_t * (2/(k+1))^((k+1)/(k-1))); % Mass flow rate
T = m_dot * sqrt(2*q*dV/m_i); % Thrust generated by the ion thruster

% Plot the results

% Plot thrust and drag vs time (normalized by orbital period)
figure
subplot(1,2,1)
plot(tt/T_orbit,T,'Color',"#0072BD",'LineWidth',5) % Plot thrust
hold on 
plot(tt/T_orbit,D,'Color',"#A2142F",'LineWidth',5,'LineStyle','--') % Plot drag
legend('Thrust','Drag','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('T,D [N]')
title ('Thrust vs Drag','Interpreter','Latex',FontSize = 33)
grid on

% Plot the absolute error between thrust and drag
subplot(1,2,2)
semilogy(tt/T_orbit, abs(T - D), 'LineWidth', 5); % Plot log scale error
xlabel('t [orbital periods]')
ylabel('$\left| T - D \right| \, [N]$')
title('Absolute error')
grid on

% Plot seismic mass position, velocity, and spool dynamics
figure
subplot(2,2,1)
plot(tt/T_orbit, xx(:,1),'Color',"#0072BD", 'LineWidth', 4) % Seismic mass position
xlabel('t [orbital periods]')
ylabel('$x_a$ [m]') 
grid on
ylim([1.7e-8,2.5e-8])
title('Seismic mass position','Interpreter','Latex',FontSize = 33)

subplot(2,2,2)
plot(tt/T_orbit, xx(:,2),'Color',"#0072BD", 'LineWidth', 4) % Seismic mass velocity
xlabel('t [orbital periods]')
ylabel('$v_a$ [m/s]')
grid on
ylim([-6e-12,6e-12])
title('Seismic mass velocity','Interpreter','Latex',FontSize = 33)

subplot(2,2,3)
plot(tt/T_orbit, xx(:,3),'Color',"#A2142F", 'LineWidth', 4) % Spool position
hold on
yline(1e-5,'LineWidth',3,'LineStyle','--') % Maximum spool position threshold
legend('$x_v$','$x_{v,max}$','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('$x_v$ [m]') 
grid on
ylim([5e-6,12.5e-6])
title('Spool position','Interpreter','Latex',FontSize = 33)

subplot(2,2,4)
hold on
plot(tt/T_orbit, xx(:,4),'Color',"#A2142F", 'LineWidth', 4) % Spool velocity
xlabel('t [orbital periods]')
ylabel('$v_v$ [m/s]') 
grid on
ylim([-5.5e-9,5.5e-9])
title('Spool velocity','Interpreter','Latex',FontSize = 33)

% Plot current over time
figure
plot(tt/T_orbit, xx(:,5), 'LineWidth', 4, 'Color', [0.3, 0.7, 0.3]) % Current over time
xlabel('t [orbital periods]')
ylabel('$I$ [A]') 
grid on
title('Current','Interpreter','Latex',FontSize = 33)


%% SIMSCAPE 

% Initial conditions for the simulation (null state)
x0_sim = [0; 0; 0; 0; 0];  

% Perform integration using ODE solver with high tolerance
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[tt,xx] = ode15s(@(t,x) rhs(t,x,var), [t0 tf], x0_sim, options);

% Compute Drag force based on the given time-dependent formula
D = (2.2 - cos(omega_s.*tt) + 1.2.*sin(omega0.*tt).*cos(omega0.*tt)) * 1e-3;

% Compute Thrust force based on ion thruster dynamics
Av = A0 + l*(l-xx(:,3)); % Valve area as a function of spool position
rho = p_t / (R*T_t); % Density of the gas in the tank
m_dot = Av * sqrt(k * rho * p_t * (2 / (k + 1))^((k + 1) / (k - 1))); % Mass flow rate
T = m_dot * sqrt(2 * q * dV / m_i); % Thrust produced by the ion thruster

% Run Simscape simulation with a fixed step size for time
out = sim('Portantiolo250184_Assign2_EX_2', 'Fixedstep','0.1');

% Plots for comparing results

% Plot Thrust and Drag forces vs time (normalized by orbital period)
figure
subplot(1,2,1)
plot(out.time/T_orbit, out.T, 'Color',"#0072BD", 'LineWidth', 5) % Plot Simscape Thrust
hold on 
plot(out.time/T_orbit, out.D, 'Color',"#A2142F", 'LineWidth', 5, 'LineStyle', '--') % Plot Simscape Drag
legend('Thrust','Drag','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('T,D [N]')
title('Thrust vs Drag','Interpreter','Latex',FontSize = 33)
grid on

% Plot error between Simscape and Matlab (ODE) results
subplot(1,2,2)
plot(out.time/T_orbit, (out.T - out.D) * 1000, 'Color', "#0072BD", 'LineWidth', 5); % Simscape error
ylim([-0.015, 0.035])
hold on
plot(tt/T_orbit, (T - D) * 1000, 'Color', "#A2142F", 'LineWidth', 5); % Matlab (ODE) error
yline(0, 'LineWidth', 3, 'LineStyle', '--') % Zero error line
legend('Simscape error','Matlab error','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('$\left( T - D \right) \, [mN]$')
title('Error')
grid on

% Plot seismic mass position and velocity vs time for both Simscape and Matlab
figure
subplot(2,2,1)
plot(out.time/T_orbit, out.xa, 'Color', "#0072BD", 'LineWidth', 4) % Simscape seismic mass position
hold on 
plot(tt/T_orbit, xx(:,1), 'Color', "#A2142F", 'LineWidth', 4, 'LineStyle', '--') % Matlab seismic mass position
legend('Simscape', 'Matlab', 'Interpreter', 'Latex', FontSize = 33)
xlabel('t [orbital periods]')
ylabel('$x_a$ [m]')
grid on
ylim([0, 2e-7])
title('Seismic mass position','Interpreter','Latex',FontSize = 33)

subplot(2,2,2)
plot(out.time/T_orbit, out.va, 'Color', "#0072BD", 'LineWidth', 4) % Simscape seismic mass velocity
hold on 
plot(tt/T_orbit, xx(:,2), 'Color', "#A2142F", 'LineWidth', 4, 'LineStyle', '--') % Matlab seismic mass velocity
xlabel('t [orbital periods]')
ylabel('$v_a$ [m/s]')
grid on
ylim([-1e-11, 4e-11])
title('Seismic mass velocity','Interpreter','Latex',FontSize = 33)

subplot(2,2,3)
plot(out.time/T_orbit, out.xv, 'Color', "#0072BD", 'LineWidth', 4) % Simscape spool position
hold on 
plot(tt/T_orbit, xx(:,3), 'Color', "#A2142F", 'LineWidth', 4, 'LineStyle', '--') % Matlab spool position
yline(1e-5, 'LineWidth', 3, 'LineStyle', '--') % Max spool position threshold
legend('','','$x_{v,max}$','Interpreter','Latex',FontSize = 33)
xlabel('t [orbital periods]')
ylabel('$x_v$ [m]')
grid on
ylim([5e-6, 12.5e-6])
title('Spool position','Interpreter','Latex',FontSize = 33)

subplot(2,2,4)
plot(out.time/T_orbit, out.vv, 'Color', "#0072BD", 'LineWidth', 4) % Simscape spool velocity
hold on
plot(tt/T_orbit, xx(:,4), 'Color', "#A2142F", 'LineWidth', 4, 'LineStyle', '--') % Matlab spool velocity
xlabel('t [orbital periods]')
ylabel('$v_v$ [m/s]')
grid on
ylim([-5.5e-9, 5.5e-9])
title('Spool velocity','Interpreter','Latex',FontSize = 33)

% Plot current vs time for both Simscape and Matlab results
figure
plot(out.time/T_orbit, out.i, 'Color', "#0072BD", 'LineWidth', 4) % Simscape current
hold on 
plot(tt/T_orbit, xx(:,5), 'Color', "#A2142F", 'LineWidth', 4, 'LineStyle', '--') % Matlab current
xlabel('t [orbital periods]')
ylabel('$I$ [A]')
grid on
title('Current','Interpreter','Latex',FontSize = 33)


%% FUNCTIONS


function K = const()
%--------------------------------------------------------------------------
% Author: Matteo Portantiolo
%
% Function to define constants related to the satellite's drag compensation system.
% These constants are used in various parts of the simulation to model 
% the components of the system, including accelerometer, amplifier, solenoid 
% valve, thruster, and drag characteristics.
%
% OUTPUT:
%   K : structure containing all model constants
%--------------------------------------------------------------------------


    % Accelerometer
    K.m_sc = 300;           % Satellite mass [kg]
    K.m_a = 0.32;           % Accelerometer mass [kg]
    K.ba_min = 1.5e3;       % Minimum damping factor for accelerometer [Ns/m]
    K.ba_max = 2e4;         % Maximum damping factor for accelerometer [Ns/m]
    K.ka_min = 5e-5;        % Minimum spring constant for accelerometer [N/m]
    K.ka_max = 3e-3;        % Maximum spring constant for accelerometer [N/m]
    K.k_acc = 1;            % Accelerometer constant

    % Amplifier
    K.Rin_min = 0.1;        % Minimum input resistance for amplifier [ohm]
    K.Rin_max = 10;         % Maximum input resistance for amplifier [ohm]
    K.Rf_min = 1e4;         % Minimum feedback resistance for amplifier [ohm]
    K.Rf_max = 8e4;         % Maximum feedback resistance for amplifier [ohm]

    % Solenoid Valve
    K.m_v = 0.1;            % Solenoid valve mass [kg]
    K.k_v = 1e3;            % Spring constant for solenoid valve [N/m]
    K.b_v = 1e3;            % Damping coefficient for solenoid valve [Ns/m]
    K.alpha = 2.1e-2;       % Solenoid valve constant
    K.beta = -60;           % Solenoid valve constant
    K.A0 = 4.7e-12;         % Initial area for thruster [m^2]
    K.l = 1e-5;             % Length parameter for solenoid valve [m]

    % Thruster
    K.k = 1.66;             % Specific constant for thruster
    K.p_t = 2e5;            % Thrust pressure [Pa]
    K.T_t = 240;            % Thrust temperature [K]
    K.R = 63.32754;         % Gas constant [J/(mol·K)]
    K.q = 1.6e-19;          % Charge of the ion [C]
    K.dV = 2000;            % Voltage drop across thruster [V]
    K.m_i = 2.188e-25;      % Ion mass [kg]

    % Drag
    K.omega_s = 1.658226e-6; % Drag frequency constant for satellite [rad/s]
    K.omega0 = 1.160758e-3;  % Drag frequency constant for satellite [rad/s]

    % Data tuning
    K.ka = 3e-3;            % Damping factor for accelerometer [N/m]
    K.ba = 1.5e3;           % Damping factor for accelerometer [Ns/m]
    K.Rin = 0.1;            % Input resistance for amplifier [ohm]
    K.Rf = 8e4;             % Feedback resistance for amplifier [ohm]

end


function dxdt = rhs(t,x,var)
%--------------------------------------------------------------------------
% Author: Matteo Portantiolo
%
% Function to define the right-hand side of the differential equations
% describing the dynamics of the satellite system, including the
% accelerometer, solenoid valve, ion thruster, and drag forces.
%
% Input:
%   t    - Current time [s]
%   x    - State vector containing the following:
%          x(1) = xa  : Position of the seismic mass [m]
%          x(2) = va  : Velocity of the seismic mass [m/s]
%          x(3) = xv  : Position of the spool [m]
%          x(4) = vv  : Velocity of the spool [m/s]
%          x(5) = i   : Current [A]
%   var  - Vector containing the following tuning parameters:
%          var(1) = ba  : Damping factor for accelerometer [Ns/m]
%          var(2) = ka  : Spring constant for accelerometer [N/m]
%          var(3) = Rin : Input resistance for amplifier [ohm]
%          var(4) = Rf  : Feedback resistance for amplifier [ohm]
%
% Output:
%   dxdt - Derivatives of the state variables, representing the dynamics
%          of the system:
%          dxdt(1) = Derivative of xa (seismic mass position) [m/s]
%          dxdt(2) = Derivative of va (seismic mass velocity) [m/s^2]
%          dxdt(3) = Derivative of xv (spool position) [m/s]
%          dxdt(4) = Derivative of vv (spool velocity) [m/s^2]
%          dxdt(5) = Derivative of i (current) [A/s]
%--------------------------------------------------------------------------

    % Initialize output vector
    dxdt = NaN(length(x), 1);

    % Constants
    K = const();  % Get constants from the 'const' function

    % Extract state variables
    xa = x(1);  % Position of the seismic mass [m]
    va = x(2);  % Velocity of the seismic mass [m/s]
    xv = x(3);  % Position of the spool [m]
    vv = x(4);  % Velocity of the spool [m/s]
    i  = x(5);  % Current [A]

    % Extract system parameters
    ba  = var(1);  % Damping factor for accelerometer [Ns/m]
    ka  = var(2);  % Spring constant for accelerometer [N/m]
    Rin = var(3);  % Input resistance for amplifier [ohm]
    Rf  = var(4);  % Feedback resistance for amplifier [ohm]

    % DYNAMICS EVOLUTION

    % Amplifier dynamics (output voltage)
    Vout = K.k_acc * va;  % Output voltage of the amplifier [V]
    Vout_mod = -Rf / Rin * Vout;  % Modified output voltage for the solenoid valve

    % Solenoid valve dynamics
    dxdt(3) = vv;  % Position change of spool (velocity)
    dxdt(4) = (-K.k_v * xv - K.b_v * vv + 0.5 * i^2 * (-K.beta / (K.alpha + K.beta * xv)^2)) / K.m_v;  % Spool acceleration
    dxdt(5) = Vout_mod / (1 / (K.alpha + K.beta * xv));  % Current change based on output voltage

    % Solenoid valve area (variable with spool position)
    Av = K.A0 + K.l * (K.l - xv);  % Area of the solenoid valve [m^2]

    % Ion thruster dynamics (thrust generation)
    rho = K.p_t / (K.R * K.T_t);  % Density of xenon gas
    m_dot = Av * sqrt(K.k * rho * K.p_t * (2 / (K.k + 1))^((K.k + 1) / (K.k - 1)));  % Mass flow rate
    T = m_dot * sqrt(2 * K.q * K.dV / K.m_i);  % Thrust generated by the ion thruster [N]

    % Drag force (time-dependent drag coefficient)
    D = (2.2 - cos(K.omega_s * t) + 1.2 * sin(K.omega0 * t) * cos(K.omega0 * t)) * 1e-3;  % Drag force [N]

    % Accelerometer dynamics (motion of seismic mass)
    dxdt(1) = va;  % Seismic mass position change (velocity)
    dxdt(2) = ((T - D) / K.m_sc * K.m_a - ba * va - ka * xa) / K.m_a;  % Seismic mass acceleration

end

