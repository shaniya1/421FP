function[CM] = compositebody_cm(mass)
%{
Self, Justin
California Polytechnic State University, SLO

The purpose of this function is to determine the location of the
centroid of a composite body. 

OUTPUTS: xcm, ycm = (location of the CM of the composite body)
INPUTS:  V = vector containing part volumes
         xbar = vector: x-distance from the origin of PART centroid to
         point of interest
         ybar = vector: y-distance from the origin of PART centroid to
         point of interest
         zbar = vector: z-distance from the origin of PART centroid to
         point of interest
%}

%% Input parameters

% x-bar of each part WITH RESPECT TO (x',y',z'). See schematic. 
% (x',y',z') = center of mass of the s/c bus. 

bus.xbar = 0;
Lpanel.xbar = 0;
Rpanel.xbar = 0;
sensor.xbar = 0;
xbar = [bus.xbar; Lpanel.xbar; Rpanel.xbar; sensor.xbar];

% ybar of each part from CM part to CM bus (x',y',z')
bus.ybar = 0;
Lpanel.ybar = -2.5;
Rpanel.ybar = 2.5;
sensor.ybar = 0;
ybar = [bus.ybar; Lpanel.ybar; Rpanel.ybar; sensor.ybar];

% zbar of each part from CM part to CM bus (x',y',z')
bus.zbar = 0;
Lpanel.zbar = 0;
Rpanel.zbar = 0;
sensor.zbar = 1.5;
zbar = [bus.zbar; Lpanel.zbar; Rpanel.zbar; sensor.zbar];

% x bar, y bar, z bar for each part in composite structure
xyz_bar = [xbar, ybar zbar];

% Now get to business
xbar = xyz_bar(:,1);
ybar = xyz_bar(:,2);
zbar = xyz_bar(:,3);

% Vixi, Aiyi, Aizi
for i = 1:length(mass)
    Vixi(i) = mass(i)*xbar(i);
    Viyi(i) = mass(i)*ybar(i);
    Vizi(i) = mass(i)*zbar(i);
end

% Finally, solving for (xbar,ybar) of COMPOSITE body
CM(1) = sum(Vixi) / sum(mass);
CM(2) = sum(Viyi) / sum(mass);
CM(3) = sum(Vizi) / sum(mass);

