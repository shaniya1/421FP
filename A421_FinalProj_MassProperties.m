%{
California Polytechnic State University, SLO
AERO421, Spring 2023
Spacecraft Attitude Dynamics and Control
Final Project Group #9

Crooks, Will
Self, Justin
Singh, Shaniya
%}
clear all; close all; clc; 

% Deliverable 1: April 14, 2023 | Mass Properties
disp("Deliverable #1: Mass Properties")
disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

%% DETUMBLE PHASE
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
mass = [500; 20; 20; 100];
totalmass = sum(mass);

% Center of mass (it is a cube)
detumble.cm = [0;0;0]; % since it is all folded up

% Inertia matrix
L = 2; % length of bus

Jx = (totalmass*L^2)/6; % equation of Moment of Inertia for cube
Jy = Jx;
Jz = Jx;

detumble.J = [Jx 0 0; 0 Jy 0; 0 0 Jz];

disp("Spacecraft total mass is: " + totalmass + " kg (DETUMBLE PHASE)")
disp("Center of mass [meters] relative to the bus center is: ")
disp(detumble.cm)
disp("Inertia Matrix [kg*m2] of s/c about CM (in DETUMBLE) is: ")
disp(detumble.J)

%% NORMAL OPERATIONS
disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
disp("PHASE II: Normal Operations (unfolded) Phase")
disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
% Spacecraft Center of Mass (NORMAL OPS)
% Lpanel is in the negative y-direction
% Rpanel is in the positive y-direction

% x-bar of each part WITH RESPECT TO (x',y',z'). See schematic. 
% (x',y',z') = center of mass of the s/c bus. 

bus.xbar = 0;
Lpanel.xbar = 0;
Rpanel.xbar = 0;
sensor.xbar = 0;

% ybar of each part from CM part to CM bus (x',y',z')
bus.ybar = 0;
Lpanel.ybar = -2.5;
Rpanel.ybar = 2.5;
sensor.ybar = 0;

% zbar of each part from CM part to CM bus (x',y',z')
bus.zbar = 0;
Lpanel.zbar = 0;
Rpanel.zbar = 0;
sensor.zbar = 1.5;

xbar = [bus.xbar; Lpanel.xbar; Rpanel.xbar; sensor.xbar];
ybar = [bus.ybar; Lpanel.ybar; Rpanel.ybar; sensor.ybar];
zbar = [bus.zbar; Lpanel.zbar; Rpanel.zbar; sensor.zbar];

% Call function for location of center of mass UNFOLDED configuration
% (NORMAL OPS)
[normal.cm] = compositebody_cm(mass,xbar,ybar,zbar);

% Moments of inertia for NORMAL OPS
% Need in vector form, so mult scalar qties by eye(3)
% PARALLEL AXIS theorem. Icm = I_part - mass*dist^2

% distances from each part to the s/c CM (body frame origin)
% TAKE INTO ACCOUNT ONLY THE PERPENDICULAR BASIS VECTORS FOR DISTANCE (per
% parallel axis theorem definition for distance part)

% xx: take only yz bases into account
bus.distxx = norm([0,0,normal.cm(3)]);
Lpanel.distxx = norm([0,2.5,-normal.cm(3)]); % (x,y,z) dist from part origin to Fb origin
Rpanel.distxx = norm([0,-2.5,-normal.cm(3)]);
sensor.distxx = norm([0,0,-(1.5-normal.cm(3))]);

% yy: take only x and z bases into account
bus.distyy = bus.distxx;
Lpanel.distyy = normal.cm(3);
Rpanel.distyy = normal.cm(3); 
sensor.distyy = sensor.distxx;

% zz: take only x and y bases into account 
bus.distzz = 0;
Lpanel.distzz = 2.5;
Rpanel.distzz = -2.5;
sensor.distzz = 0;

% Computs components of J matrix individually
Jxx.bus = (1/6)*(mass(1)*L^2) + mass(1)*bus.distxx^2; % I_cube eqn
Jxx.Lpanel = (1/12)*(mass(2)*(3^2+0.05^2)) + mass(2)*Lpanel.distxx^2;
Jxx.Rpanel = (1/12)*(mass(3)*(3^2+0.05^2)) + mass(3)*Rpanel.distxx^2;
Jxx.sensor = (1/12)*(mass(4)*(0.25^2+1^2)) + mass(4)*sensor.distxx^2; % using rect prism Ixx

Jyy.bus = Jxx.bus;
Jyy.Lpanel = (1/12)*(mass(2)*(0.05^2+2^2)) + mass(2)*Lpanel.distyy^2;
Jyy.Rpanel = (1/12)*(mass(3)*(0.05^2+2^2)) + mass(3)*Rpanel.distyy^2;
Jyy.sensor = Jxx.sensor;

Jzz.bus = (1/6)*(mass(1)*L^2) + mass(1)*bus.distzz^2;
Jzz.Lpanel = (1/12)*(mass(2)*(3^2+2^2)) + mass(2)*Lpanel.distzz^2;
Jzz.Rpanel = (1/12)*(mass(2)*(3^2+2^2)) + mass(2)*Rpanel.distzz^2;
Jzz.sensor = (1/12)*(mass(4)*(0.25^2+0.25^2)) + mass(4)*sensor.distzz^2;

% Put it all together
Jxx.total = Jxx.bus + Jxx.Lpanel + Jxx.Rpanel + Jxx.sensor;
Jyy.total = Jyy.bus + Jyy.Lpanel + Jyy.Rpanel + Jyy.sensor;
Jzz.total = Jzz.bus + Jzz.Lpanel + Jzz.Rpanel + Jzz.sensor;
J.normal = [Jxx.total 0 0; 0 Jyy.total 0; 0 0 Jzz.total];

% Print results
disp("Spacecraft mass is: " + totalmass + " kg (NORMAL PHASE)")
disp("Center of mass [meters] for NORMAL OPERATIONS relative to bus center is: ")
disp(normal.cm')

disp("Inertia Matrix [kg*m2] of s/c about CM (in NORMAL OPS) is: ")
disp(J.normal)