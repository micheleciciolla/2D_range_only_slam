close all
clear
clc

% addpath '../'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'


% The dataset is composed by a g2o file which contain poses and range observations. 
% The file contain also odometry edges that are used to construct the initial guess
% for the problem.

% [landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_ground_truth.g2o');


return



% simulation cycle
for i=1:length(transitions)

  transitions_t  = transitions(i);
  observations_t = observations(i);
  poses_t = poses(i);
  landmarks_t = landmarks(i);
  
	printf(': [%f, %f, %f, %f]\n',transitions_t, observations_t,poses_t, landmarks_t  );
	fflush(stdout);


endfor
