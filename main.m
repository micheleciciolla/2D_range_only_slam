close all
clear
clc
warning off;
pause(1);

addpath '../'
addpath 'tools'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'
addpath './tools/getIDs_and_index.m'
addpath './tools/get_formatted_index.m'
addpath './tools/initial_guess_eval.m'
addpath './tools/get_initial_guess.m'

%% ------------------------------------ %%
%% ----------- START ------------------ %%
%% ------------------------------------ %%
printf("\n\n%% ------------------------------------ %%\n%% -------- Importing Datasets -------- %%\n%% ------------------------------------ %%\n\n");
fflush(stdout);

% The dataset is composed by a g2o file which contain poses and range observations. 
% The file contain also odometry edges that are used to construct the initial guess
% for the problem.

% LOADING BOTH DATASETS
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
[landmarks_ground_truth, poses_ground_truth, transitions_ground_truth, observations_ground_truth] = loadG2o('slam2d_range_only_ground_truth.g2o');

%% ------------------------------------ %%
%% ----------- INITIAL GUESS ---------- %%
%% ------------------------------------ %%
printf("\n\n%% ------------------------------------ %%\n%% ----- Generating an initial guess -- %%\n%% ------------------------------------ %%\n\n");
fflush(stdout);

% here i'm getting a LIST of HOW MANY landarmsks and their ID
available_landmarks = zeros(length(landmarks_ground_truth),1);
for l=1:length(landmarks_ground_truth)
	available_landmarks(l) = landmarks_ground_truth(l).landmark_id;
endfor

%% get an INITIAL GUESS of the landmarks given their id, poses and obs from init_guess_dataset
% FIELDS: 
% 	landmarks(i).id
% 	landmarks(i).landmark_position(1) and landmarks(i).landmark_position(2)
landmarks = get_initial_guess(available_landmarks, poses, observations);

%% ------------------------------------ %%
%% ----------- ICP INIT --------------- %%
%% ------------------------------------ %%

global num_poses = length(poses);
global num_landmarks = length(landmarks);

% (1)
% BUILD XR_guess and XR_true
XR_guess = zeros(3,3,num_poses);
XR_true = zeros(3,3,num_poses);

for p=1:num_poses

	%%%%%%%%%%% XR_guess %%%%%%%%%%%
	% robot coordinates for that pose p
	[x,y,theta] = deal(poses(p).x, poses(p).y, poses(p).theta);

	% rotz 
	XR_guess(1,1,p) = cos(theta);
	XR_guess(1,2,p) = -sin(theta);
	XR_guess(2,1,p) = sin(theta);
	XR_guess(2,2,p) = cos(theta);
	
	% t
	XR_guess(1,3,p) = x;
	XR_guess(2,3,p) = y;
	
	% 1
	XR_guess(3,3,p) = 1;
	
	%%%%%%%%%%% XR_true %%%%%%%%%%%
	
	% robot coordinates for that pose p
	[x,y,theta] = deal(poses_ground_truth(p).x, poses_ground_truth(p).y, poses_ground_truth(p).theta);

	% rotz 
	XR_true(1,1,p) = cos(theta);
	XR_true(1,2,p) = -sin(theta);
	XR_true(2,1,p) = sin(theta);
	XR_true(2,2,p) = cos(theta);
	
	% t
	XR_true(1,3,p) = x;
	XR_true(2,3,p) = y;
	
	% 1
	XR_true(3,3,p) = 1;

endfor

% (2)
% BUILD XL_guess and XL_true
XL_guess = zeros(2,num_landmarks);
XL_true = zeros(2,num_landmarks);

for l=1:num_landmarks
	XL_guess(1:2,l) = landmarks(l).landmark_position;
	XL_true(1:2,l) = [landmarks_ground_truth(l).x_pose; landmarks_ground_truth(l).y_pose];
endfor

% Evaluating how good is your XL guess
eval_guess = initial_guess_eval(XL_true,XL_guess);
fflush(stdout);

% (3)
% BUILD Z AND ASSOCIATIONS
num_measurements = 0;
for o=1:length(observations)
	% num_measurements is the total amount of measuremtns you got
	num_measurements += length(observations(o).observation);
endfor

% building associations and Z
% associations is a 2x num_measurements vector in which we have [ID_pose, ID_landmark]
% so for each pose you have all the landmark_id you see from there

% Z is just all the range you measure
associations = zeros(2,num_measurements);
Z = zeros(1,num_measurements);

% init idices
poseID = 1;
measure_num = 1;
% get a ordered list of landmark_id and their order position
allIDs = getIDs_and_index(observations);

for o=1:length(observations)
		
		for i=1:length(observations(o).observation)
		
			associations(1,measure_num) = poseID;
			% real id must be converted in [1,..61] to match ICP indices
			% 90 is not acceptable for example (see getIDs_and_index.m)
			real_id = observations(o).observation(i).landmark_id;
			associations(2,measure_num) = get_formatted_index(allIDs,real_id);

			Z(1,measure_num) = observations(o).observation(i).range_measure;
			
			measure_num+=1;
		endfor
		poseID+=1;
	
endfor


%% ------------------------------------ %%
%% ----------- ICP CALL --------------- %%
%% ------------------------------------ %%
printf("\n\n%% ------------------------------------ %%\n%% ------ Starting ICP optimization --- %%\n%% ------------------------------------ %%\n\n");
fflush(stdout);

source "./multi_ICP_3d.m"

[XR_correction, XL_correction, chi_stats, num_inliers]=doMultiICP(...
							XR_guess, XL_guess, Z,
							
							associations, 
							num_poses, 
							num_landmarks,
							
							num_iterations=10, 
							damping=0.15, 
							kernel_threshold=2.0);
							

printf("\n\n%% ------------------------------------ %%\n%% ------- ICP optimization DONE ------ %%\n%% ------------------------------------ %%\n\n");
fflush(stdout);
pause(1);

% Evaluating how good is your XL correction
eval_correction = initial_guess_eval(XL_true,XL_correction);
fflush(stdout);
printf("\nAt guess stage %i were verified landmarks\nAt correction stage %i are verified landmarks\n",[eval_guess,eval_correction]);
fflush(stdout);

%% ------------------------------------ %%
%% ---------- TRAJECTORY -------------- %%
%% ------------------------------------ %%
%% -- PLOT GROUND TRUTH - CORRECTION -- %%
%% ------------------------------------ %%

figure()
title("FINAL RESULTS");
subplot(1,2,1);
grid minor;
hold on;
xlim([-15,15]);
ylim([-15,10]);
plot(XR_true(1,3,:),XR_true(2,3,:), 'g-.', 'linewidth', 3);
plot(XR_correction(1,3,:),XR_correction(2,3,:), 'b-', 'linewidth', 3);
plot(XR_guess(1,3,:),XR_guess(2,3,:), 'k-.', 'linewidth', 3);
legend("ground truth","correction","guess");
pause(0.5)

%{
subplot(1,2,2);
hold on;
title("trajectory");
plot(XR(1,3,:),XR(2,3,:), 'b-', 'linewidth', 3);
plot(XR_guess(1,3,:),XR_guess(2,3,:), 'r-', 'linewidth', 3);
legend("correction","guess");
pause(0.5)
%}


%% ------------------------------------ %%
%% ---------------- MAP --------------- %%
%% ------------------------------------ %%
%% -- PLOT GROUND TRUTH - CORRECTION -- %%
%% ------------------------------------ %%

% figure()
% subplot(1,2,1);
subplot(1,2,2);

hold on;
xlim([-15,15]);
ylim([-15,10]);
grid minor;
plot(XL_correction(1,:),XL_correction(2,:), "o","markersize",7,"color",'k',"markerfacecolor",'b','linestyle','none');
plot(XL_true(1,:),XL_true(2,:), "o","markersize",7,"color",'k',"markerfacecolor",'g','linestyle','none');
% plot(XL_guess(1,:),XL_guess(2,:),"o","markersize",7,"color",'k',"markerfacecolor",'none','linestyle','none');
legend("correction","ground truth","guess");
pause(0.5)

%{
subplot(1,2,2);
hold on;
title("correction");
plot(XL(1,:),XL(2,:),  "o","markersize",7,"color",'k',"markerfacecolor",'b','linestyle','none');
plot(XL_guess(1,:),XL_guess(2,:),"o","markersize",7,"color",'k',"markerfacecolor",'r','linestyle','none');
legend("correction","guess");
pause(0.5)
%}

figure();
hold on;
title("RESULTING TRAJECTORY");
plot(XR_true(1,3,:),XR_true(2,3,:), 'g-', 'linewidth', 3);
plot(XR_correction(1,3,:),XR_correction(2,3,:), 'b-', 'linewidth', 3);
legend("ground truth","correction");

printf("\n\n%% ------------------------------------ %%\n%% ---------- PLOT GENERATED ---------- %%\n%% ------------------------------------ %%\n\n");
fflush(stdout);
