function [Op, Or] = get_params(x, N)
% FILE: get_params.m measures the polarization and rotation orders
%
% DESCRIPTION: 
% The Polarization (Op) and Rotation (Or) orders are 
% paramters that describe the swarm, as a whole. Op and Or are both between
% 0 and 1. A high Op indicates that high degree of alignment between all 
% the agents in the swarm. A high Or indicates a high rotating motion of
% the swarm about its center of mass.
%
% INPUTS:
% 1. N - Number of robots
% 2. x - the positions and direction of the robots
%
% OUTPUTS:
% 1. Op - Polarization order (scalar between 0 and 1) 
% 2. Or - Rotation order (scalar between 0 and 1)
%
% TODO:
% None

%% Authors: Safwan Alam, Musad Haque - 2018 
%%%%%%%%%%%%%

%Cumulative ui and ri x ui
Cu = [0;0;0];
Cru = [0;0;0];

% Swarm center of mass centroid [xc; yc]
rho = [sum(x(1,:),2)/N;sum(x(2,:),2)/N];

for i=1:1:N
    ui = [cos(x(3, i));sin(x(3, i)); 0];
    Cu = Cu + ui;
    ri = [x(1,i) - rho(1,1); x(2,i) - rho(2,1); 0]...
        /norm([x(1,i) - rho(1,1); x(2,i) - rho(2,1); 0]);
    Cru = Cru + cross(ui, ri);
end
Op = norm(Cu)/N;
Or = norm(Cru)/N;

end


