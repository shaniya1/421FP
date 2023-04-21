function [J_normal] = find_J_normalops(mass,busLength,CM_Normal)

%{
Description

%}
% Define inputs
L = busLength; % length of s/c bus
normal.cm = CM_Normal;

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

distxx = [bus.distxx;Lpanel.distxx; Rpanel.distxx; sensor.distxx];

% yy: take only x and z bases into account
bus.distyy = bus.distxx;
Lpanel.distyy = normal.cm(3);
Rpanel.distyy = normal.cm(3); 
sensor.distyy = sensor.distxx;

distyy = [bus.distyy;Lpanel.distyy; Rpanel.distyy; sensor.distyy];

% zz: take only x and y bases into account 
bus.distzz = 0;
Lpanel.distzz = 2.5;
Rpanel.distzz = -2.5;
sensor.distzz = 0;

distzz = [bus.distzz;Lpanel.distzz; Rpanel.distzz; sensor.distzz];

distVect = [distxx, distyy, distzz];

distxx = distVect(:,1);
distyy = distVect(:,2);
distzz = distVect(:,3);

bus.distxx = distxx(1);
Lpanel.distxx = distxx(2);
Rpanel.distxx = distxx(3);
sensor.distxx = distxx(4);

bus.distyy = distyy(1);
Lpanel.distyy = distyy(2);
Rpanel.distyy = distyy(3);
sensor.distyy = distyy(4);

bus.distzz = distzz(1);
Lpanel.distzz = distzz(2);
Rpanel.distzz = distzz(3);
sensor.distzz = distzz(4);

% Computes components of J matrix individually
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

% final ouput
J_normal = [Jxx.total 0 0; 0 Jyy.total 0; 0 0 Jzz.total];

end