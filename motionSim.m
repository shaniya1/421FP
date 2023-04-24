function [stateNew] = motionSim(time,state_init,I,torque)
% Inputs
    % time
    % state init = [wx,wy,wz,phi,theta,psi,epsilonx,epsilony,epsilonz,eta]
    % I = inertia matrix
    % torque
% Ouput
    % stateNew = derivative of stateInit
% *All angles in RADIANS*

% Angular Velocity
w_init = state_init(1:3);    
wx = w_init(1);
wy = w_init(2);
wz = w_init(3);
wcross = [0, -wz, wy; wz, 0, -wx; -wy, wx, 0];

w_dot = (I^-1)*(-wcross*I*w_init+torque);

% Euler Angles
initEuler = state_init(4:6);
phi = initEuler(1);
theta = initEuler(2);
psi = initEuler(3);

[Euler_dot] = (1/cos(theta))*...
    [cos(theta), sin(phi)*sin(theta), cos(phi)*sin(theta);...
     0,          cos(phi)*cos(theta), -sin(phi)*cos(theta);...
     0,          sin(phi), cos(phi)]*w_init;

% Quaternion
epsilon0 = state_init(7:9);
eta0 = state_init(10);
epsilCross = [          0, -epsilon0(3),  epsilon0(2); ...
              epsilon0(3),            0, -epsilon0(1); ...
             -epsilon0(2),  epsilon0(1),            0];

epsilonDot = 0.5*(eta0*eye(3) + epsilCross)*w_init;
etaDot = -0.5*epsilon0'*w_init;

stateNew = [w_dot;Euler_dot;epsilonDot;etaDot];
end