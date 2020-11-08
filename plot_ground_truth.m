close all
clear
clc

addpath '../'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'


% The dataset is composed by a g2o file which contain poses and range observations. 
% The file contain also odometry edges that are used to construct the initial guess
% for the problem.

[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_ground_truth.g2o');

%init graphics
figure(1); title("ekf-localization-range-only");



% plot all poses
for p=1:length(poses)
	x = poses(p).x;
	y = poses(p).y;
	
	pause(.1);

	plot(x,y,'linestyle',":",'marker','square','color','r','linewidth',3,'interruptible',"off")
	hold on
endfor

% plot all the landmarks
for l=1:length(landmarks)
	x = landmarks(l).x_pose;
	y = landmarks(l).y_pose;
	pause(.1);

	plot(x,y,'o','color','k','markerfacecolor','k','markersize',6)
	hold on
endfor