function [initialState,Torque,tspan,InertiaMatrix] = A421_torqueFreeMotion(InertiaMatrix)
% normal operations for 1 orbit
period_sec = 100*60; % [min] to [sec]

W_b_ECI = [0.001; -0.001; 0.002]; % rad/sec
epsilon_b_LVLH = [0 0 0]';
eta_b_LVLH = 1;

% Find r and v in ECI from COEs
RAAN_rad = 0;
inc_rad = deg2rad(98.43);
w_rad = 0;
h_km2PerSec = 53335.2;
ecc = 0; % circular orbit
theta_rad = 0;

[r_ECI,v_ECI] = r_and_v_from_COEs(RAAN_rad,inc_rad,w_rad,h_km2PerSec,ecc,theta_rad);

% Find x,y,z of LVLH in ECI frame
z_LVLH = -r_ECI/norm(r_ECI);
y_LVLH = -cross(r_ECI,v_ECI)/norm(cross(r_ECI,v_ECI));
x_LVLH = cross(y_LVLH,z_LVLH);

% Find LVLH-ECI rotation matrix
x_ECI = [1 0 0]';
y_ECI = [0 1 0]';
z_ECI = [0 0 1]';
C_LVLH_ECI = [dot(x_LVLH,x_ECI), dot(x_LVLH,y_ECI), dot(x_LVLH,z_ECI);...
              dot(y_LVLH,x_ECI), dot(y_LVLH,y_ECI), dot(y_LVLH,z_ECI);...
              dot(z_LVLH,x_ECI), dot(z_LVLH,y_ECI), dot(z_LVLH,z_ECI)];

% Find b-LVLH rotation matrix
epsilon_b_LVLH_cross = [0, -epsilon_b_LVLH(3), epsilon_b_LVLH(2);...
                        epsilon_b_LVLH(3), 0, -epsilon_b_LVLH(1);...
                        -epsilon_b_LVLH(2), epsilon_b_LVLH(1), 0];
C_b_LVLH = (2*eta_b_LVLH^2 - 1)*eye(3) + 2*(epsilon_b_LVLH*epsilon_b_LVLH') - 2*eta_b_LVLH*epsilon_b_LVLH_cross;

% b_LVLH euler angles
princpRot_b_LVLH = [atan2(C_b_LVLH(2,3),C_b_LVLH(3,3));...
                    -asin(C_b_LVLH(1,3));...
                    atan2(C_b_LVLH(1,2),C_b_LVLH(1,1))];

% LVLH_ECI euler angles
princpRot_LVLH_ECI = [atan2(C_LVLH_ECI(2,3),C_LVLH_ECI(3,3));...
                    -asin(C_LVLH_ECI(1,3));...
                    atan2(C_LVLH_ECI(1,2),C_LVLH_ECI(1,1))];

% b_ECI angles
princpRot_b_ECI = princpRot_LVLH_ECI + princpRot_b_LVLH;

% b_ECI rotation matrix
C_b_ECI = C_b_LVLH*C_LVLH_ECI;

% b_ECI quaternion
eta_b_ECI = 0.5*sqrt(trace(C_b_ECI)+1);
epsilon_b_ECI = [(C_b_ECI(2,3)-C_b_ECI(3,2))/(4*eta_b_ECI);...
                 (C_b_ECI(3,1)-C_b_ECI(1,3))/(4*eta_b_ECI);...
                 (C_b_ECI(1,2)-C_b_ECI(2,1))/(4*eta_b_ECI)];

tspan = [0 period_sec];

initialState = [W_b_ECI;princpRot_b_ECI;epsilon_b_ECI;eta_b_ECI];

Torque = 0; % torque free motion

% tolerance
options = odeset('RelTol',1e-8,'AbsTol',1e-8);

%% UNCOMMENT this section to solve the ODE without using simulink
% [tnew,StateNew] = ode45(@motionSim,tspan,initialState,options,InertiaMatrix,Torque);
% 
% figure
% subplot(3,1,1)
% plot(tnew,StateNew(:,1))
% hold on
% plot(tnew,StateNew(:,2))
% plot(tnew,StateNew(:,3))
% title("Angular Velocity as Function of Time")
% xlabel("Time [sec]")
% ylabel("Angular Velocity [rad/sec]")
% legend("\omega_x","\omega_y","\omega_z",'Location','eastoutside')
% grid on
% 
% subplot(3,1,2)
% plot(tnew,StateNew(:,7))
% hold on
% plot(tnew,StateNew(:,8))
% plot(tnew,StateNew(:,9))
% plot(tnew,StateNew(:,10))
% title("Quaternion as Function of Time")
% xlabel("Time [sec]")
% ylabel("Quaternions")
% legend("\epsilon_x","\epsilon_y","\epsilon_z","\eta",'Location','eastoutside')
% grid on
% 
% subplot(3,1,3)
% plot(tnew,rad2deg(StateNew(:,4))) % convert from [rad] to [deg]
% hold on
% plot(tnew,rad2deg(StateNew(:,5)))
% plot(tnew,rad2deg(StateNew(:,6)))
% title("Euler Angles as Functions of Time")
% xlabel("Time [sec]")
% ylabel("Angle [deg]")
% legend("\phi(t)","\theta(t)","\psi(t)",'Location','eastoutside')
% grid on
% sgtitle("MATLAB Solved Plots")