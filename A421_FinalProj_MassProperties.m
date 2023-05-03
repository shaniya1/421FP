% function A421_FinalProj_MassProperties
%{
Self, Justin
California Polytechnic State University, SLO
AERO421, Spring 2023
Spacecraft Attitude Dynamics and Control
Final Project Group #9

Crooks, Will
Self, Justin
Singh, Shaniya
%}
clear all; close all; clc; 

%% Introduction to the Main Script
%{
This script provides the framework for the AERO 421 Final Project.
Outside functions are called within this script.
The script is organized by section by deliverable in chronological order.
%}

%% Deliverable 1: April 14, 2023 | Mass Properties
disp("Deliverable #1: Mass Properties")
disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")


% DETUMBLE PHASE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


disp("PHASE I: Detumble Phase")
disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
%{
Mass properties, kg
MASS MATRIX DESCRIPTION: 
1: bus
2: L solar panel
3: R solar panel
4: sensor
%}
mass = [500; 20; 20; 100]; % knowns
totalmass = sum(mass);
L = 2; % length of bus

% Call function to find CM and J matrix for DETUMBLE phase
[detumble.cm,detumble.J] = A421_FinalProj_MassProperties_function(L,totalmass);

% Print results for DETUMBLE phase
disp("Spacecraft total mass is: " + totalmass + " kg (DETUMBLE PHASE)")
disp("Center of mass [meters] relative to the bus center is: ")
disp(detumble.cm)
disp("Inertia Matrix [kg*m2] of s/c about CM (in DETUMBLE) is: ")
disp(detumble.J)


% NORMAL OPERATIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
disp("PHASE II: Normal Operations (unfolded) Phase")
disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
% Spacecraft Center of Mass (NORMAL OPS)
% Lpanel is in the negative y-direction
% Rpanel is in the positive y-direction

% Call function for location of center of mass UNFOLDED configuration
% (NORMAL OPS)
[normal.cm] = compositebody_cm(mass);

% Call function to find J_Normal_Operations
[normal.J] = find_J_normalops(mass,L,normal.cm);

% Print results
disp("Spacecraft mass is: " + totalmass + " kg (NORMAL PHASE)")
disp("Center of mass [meters] for NORMAL OPERATIONS relative to bus center is: ")
disp(normal.cm')

disp("Inertia Matrix [kg*m2] of s/c about CM (in NORMAL OPS) is: ")
disp(normal.J)

%% Deliverable 2: April 21, 2023 | Torque-Free Motion
[initialState,Torque,tspan,InertiaMatrix] = A421_torqueFreeMotion(normal.J);

simData = sim("A421_FP_pt2_model.slx");

angVeloc = simData.ScopeData.signals(1).values; % [rad/sec]
eulerAng = simData.ScopeData.signals(2).values; % [rad]
quaternion = simData.ScopeData.signals(3).values;
timeVect = simData.tout;

% Plots
figure
subplot(3,1,1)
plot(timeVect,angVeloc)
title("Angular Velocity as Function of Time")
xlabel("Time [sec]")
ylabel("Angular Velocity [rad/sec]")
legend("\omega_x","\omega_y","\omega_z",'Location','eastoutside')
grid on

subplot(3,1,2)
plot(timeVect,quaternion)
title("Quaternion as Function of Time")
xlabel("Time [sec]")
ylabel("Quaternions")
legend("\epsilon_x","\epsilon_y","\epsilon_z","\eta",'Location','eastoutside')
grid on

subplot(3,1,3)
plot(timeVect,rad2deg(eulerAng))
title("Euler Angles as Functions of Time")
xlabel("Time [sec]")
ylabel("Angle [deg]")
legend("\phi(t)","\theta(t)","\psi(t)",'Location','eastoutside')
grid on
sgtitle("Simulink Solved Plots")

%% Deliverable 3: May 5, 2023 | Detumble Simulation

detumble.w_b_ECI = [-0.05;0.03;0.2]; % rad/sec
detumble.initialState = [detumble.w_b_ECI;initialState(4:10)];
detumble.k = 0.2; % Gain Selection code needed
detumble.deltaT = 5*100*60; % sec

detumbleSim = sim("A421_FP_pt3_model.slx");

detumble.angVeloc = detumbleSim.ScopeData.signals(1).values; % [rad/sec]
detumble.eulerAng = detumbleSim.ScopeData.signals(2).values; % [rad]
detumble.quaternion = detumbleSim.ScopeData.signals(3).values;
detumble.torque = detumbleSim.ScopeData.signals(4).values; % N
detumble.timeVect = detumbleSim.tout;

%% Plots
figure
subplot(3,1,1)
plot(detumble.timeVect,detumble.angVeloc)
title("Angular Velocities")
xlabel("Time [sec]")
ylabel("Angular Velocity [rad/sec]")
legend("\omega_x","\omega_y","\omega_z",'Location','eastoutside')
grid on

subplot(3,1,2)
plot(detumble.timeVect,detumble.quaternion)
title("Quaternions")
xlabel("Time [sec]")
ylabel("Quaternion Parameter")
legend("\epsilon_x","\epsilon_y","\epsilon_z","\eta",'Location','eastoutside')
grid on

subplot(3,1,3)
plot(detumble.timeVect,rad2deg(detumble.eulerAng))
title("Euler Angles")
xlabel("Time [sec]")
ylabel("Angle [deg]")
legend("\phi(t)","\theta(t)","\psi(t)",'Location','eastoutside')
grid on

% subplot(4,1,4)
figure
plot(detumble.timeVect,detumble.torque)
title("Thruster Torque")
xlabel("Time [sec]")
ylabel("Troque [N]")
legend("T_x","T_y","T_z",'Location','eastoutside')
grid on