% compute initial guess for landmarks positions in world coordinateclose all
clear all
clc
pause(0.1)

addpath '../'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'

% The dataset is composed by a g2o file which contain poses and range observations. 
% The file contain also odometry edges that are used to construct the initial guess
% for the problem.

[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');

% VERTEX_XY 19 -0.297143 4.91945

% prendi la prima posizione del robot dove vede il landmark

real_pos = [-0.297143 ; 4.91945];

xr = 0.43;
yr = 0.94;

initial_range = 4.8; % your measure

xl = xr+initial_range;
yl = yr+initial_range;

positions_x = [0.43, 0.53, 1.4, 0.79];
positions_y = [0.94, 1.9, 1.7, 3.98];
epsi = [xl; yl];

measurements = [4.8, 3.93, 4.16,1.96 ];

figure()
plot(epsi(1),epsi(2),'*',"markersize",10,"color",'k');
hold on
plot(real_pos(1),real_pos(2),'*',"markersize",10,"color",'r');

for i=1:4
	
	xl = epsi(1);
	yl = epsi(2);
	
	xr = positions_x(i);
	yr = positions_y(i);
	
	measurement = measurements(i);
	
	current_r_square = xl^2 + xr^2 - 2*xl*xr + yl^2 + yr^2 - 2*yl*yr;
	regression = [2*epsi(1) - 2*xr, 2*epsi(2) - 2*yr];
		
	error = measurement - sqrt(current_r_square);
	epsi = pinv(regression)*abs(error);
	plot(epsi(1),epsi(2),'*',"markersize",10,"color",'k');
	pause(0.5);

	
endfor
fprintf('result %f \n',epsi - real_pos);

