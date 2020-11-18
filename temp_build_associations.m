close all
clear
clc

addpath '../'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'



%% ------------------------------------ %%
%% ----------- START ------------------ %%
%% ------------------------------------ %%

% The dataset is composed by a g2o file which contain poses and range observations. 
% The file contain also odometry edges that are used to construct the initial guess
% for the problem.

% loading both datasets
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
num_measurements = 1780;



associations = zeros(2,num_measurements);
Z = zeros(1,num_measurements);

% init idices
poseID = 1;
measure_num = 1;


allIDs = [];

for o=1:length(observations)

for i=1:length(observations(o).observation)

	gottenID = observations(o).observation(i).landmark_id;
	allIDs(1,end+1) = gottenID;
	
endfor

endfor
% sort and remove all clones
allIDs = sort(unique(allIDs));

for i=1:length(allIDs)
	allIDs(2,i) = i;
endfor

allIDs
			
			
			
 


return


Z = zeros(1,num_measurements);

% build associations
associations = zeros(2,num_measurements);

measurement_num=1;
for (pose_num=1:num_poses)
		% pose_num from 1 to 
    for (landmark_num=1:num_landmarks)
			% landmark_num from 1 to 61
			
			% update associations
			associations(:,measurement_num)=[pose_num,landmark_num]';
			
			% plug in measurement
			Z(:,measurement_num)= norm(t-Xl(1:2)); % edit
			measurement_num++;
    endfor;
endfor