function [detumble_CM,detumble_J] = A421_FinalProj_MassProperties_function(busLength,totalmass)
%{
Description
%}

% Center of mass (it is a cube)
detumble_CM = [0;0;0]; % since it is all folded up
L = busLength;

% Inertia matrix
Jx = (totalmass*L^2)/6; % equation of Moment of Inertia for cube
Jy = Jx;
Jz = Jx;

detumble_J = [Jx 0 0; 0 Jy 0; 0 0 Jz];

end % function