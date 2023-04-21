% function A421_FinalProject_Team9
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
1. Mass Properties
2. Torque Free Motion
3. Detumble Simulation
4. Disturbance Simulation
5. Reaction Wheel Control
6. Reaction Wheel Sizing
7. Visualization
8. Final Reflection
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
A421_torqueFreeMotion(normal.J)
% Stuff here.