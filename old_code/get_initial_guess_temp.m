% compute initial guess for landmarks positions in world coordinateclose all
clear all
close all
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

% [landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_ground_truth.g2o');

[landmarks_gt, poses_gt, transitions_gt, observations_gt] = loadG2o('slam2d_range_only_ground_truth.g2o');


% create a list of all the poses from which i see the first landmark

function id_pose_range = find_landmark_references(searched_landmark,poses,observations)
	
	id_pose_range = [];
	land_poses_id = [];
	land_poses_xy = [];
	land_ranges = [];

	count = 0;

	for s=1:length(observations)
	% for each sample in observations
		sample = observations(s);
		
		for o=1:length(sample.observation)
		
			landmark_and_range = sample.observation(o);
			landmark_seen = landmark_and_range.landmark_id;
			
			if landmark_seen == searched_landmark
				count = count+1;
				% add pose and range to the lists
				land_poses_id(end+1) = sample.pose_id;
				land_ranges(end+1) = landmark_and_range.range_measure;
			
			endif
		endfor
	endfor

	% printf('\nlandmark %i has been seen from %i poses\n', [searched_landmark, count]);
	
	% retrieve each xy couple from any pose_id in land_poses_id in poses from G2o

	for i=1:length(land_poses_id)
		for p=1:length(poses)

			if poses(p).id == land_poses_id(i)
			% tell me xy
				land_poses_xy(end+1).x = poses(p).x;
				land_poses_xy(end).y = poses(p).y;
			endif

		endfor
	endfor
	
	% printf('\nland_poses_id has length %i\n', length(land_poses_id));
	% printf('\nland_poses_xy has length %i\n', length(land_poses_xy));
	% printf('\nland_ranges has length %i\n', length(land_ranges));
		
	% save a prettier data structure
	for e=1:count
	
		id_pose_range(e).id = land_poses_id(e);
		id_pose_range(e).pose = land_poses_xy(e);
		id_pose_range(e).range = land_ranges(e);
		
	endfor
	% add info how many times you've seen it (just first position)
	id_pose_range(1).counts = count;
end

function land_position = converge_landmark_pos(info)

	starting_xr = info(1).pose.x;
	starting_yr = info(1).pose.y;
	starting_range = info(1).range;
	samples_size = info(1).counts;
	
	%{
	if samples_size < 3.0
			printf("\n The landmark %i has < 3 samples\n", info.id); fflush(stdout);
	endif
	%}
	randn = rand(1);
	randn2 = rand(1);
	
	if randn > 0.5 
		starting_xl = starting_xr + randn*starting_range;
	else 		
		starting_xl = starting_xr - randn*starting_range;
	endif
	
	if randn2 > 0.5 
		starting_yl = starting_yr + (1-randn^2)*starting_range;
	else 		
		starting_yl = starting_yr - (1-randn^2)*starting_range;
	endif
		
	epsi = [starting_xl; starting_yl];
	xl = epsi(1); yl = epsi(2);
	
	% initial guess
	% plot(epsi(1),epsi(2),'square',"markersize",7,"color",'b',"markerfacecolor",'b');

	%% defining error vector, regression matrix, measurements
	
	error = zeros(samples_size,1);
	regression = zeros(samples_size,2);
	real_r = zeros(samples_size,1);
	r = zeros(samples_size,1);
	
	delta_epsi = zeros(size(epsi));
	% accumulate the matrix
	for i=2:samples_size % all the time you've seen the landmark
		xl = epsi(1); yl = epsi(2);

		% retrieve position of robot at the id_pose
		xr = info(i).pose.x;
		yr = info(i).pose.y;
		% retrieve range measure at that id_pose
		real_r(i) = info(i).range;
		
		% calculate range^2 based on position of robot and landmark
		r(i) = xl^2 + xr^2 - 2*xl*xr + yl^2 + yr^2 - 2*yl*yr;
		
		% accumulate in the matrix
		regression(i,:) = [2*xl - 2*xr, 2*yl - 2*yr];
		
		% accumulate error
		error(i) =  real_r(i).^2 - r(i);
		
		% optmizing one time
		
		% never do global otpmization
		if mod(i,96)==0
			delta_epsi = pinv(regression)*((error));
			epsi = epsi + delta_epsi;
		else
			delta_epsi = pinv(regression(i,:))*error(i);
			epsi = epsi + delta_epsi;
		endif
		
		
	endfor

	fflush(stdout);
	% optmizing one time
	%delta_epsi = pinv(regression)*((error));
	%land_position = epsi + delta_epsi;
	land_position = epsi;


end

%% ------------------------------------ %%
%% ------ Generating an initial guess - %%
%% ------------------------------------ %%

printf("\n\n%% ------------------------------------ %%\n%% ------Generating an initial guess -- %%\n%% ------------------------------------ %%\n\n");


my_initial_condition = [];

printf('Dataset processed %d%% \n',0);
for l=1:length(landmarks_gt)

	if mod( round(l/length(landmarks_gt)*100), 33)==0.0
		printf('Dataset processed %d%% \n',[round(l/length(landmarks_gt)*100)]);
		fflush(stdout);
	endif
	
	% now i'm optimizing this landmark id
	searched_landmark = landmarks_gt(l).landmark_id;
	
	% retrieve landmark info from the dataset 
	% you get landmark_position, range, id, how many times you got it
	landmark_info = find_landmark_references(searched_landmark,poses,observations);
	% start optimization process for this landmark based on the info you have
	landmark_pos = converge_landmark_pos(landmark_info);
	
	% update initial_condition
	my_initial_condition(end+1).id = searched_landmark;
	my_initial_condition(end).landmark_position = landmark_pos;

endfor
printf('Dataset processed %d%% \n',100);
fflush(stdout);
pause(0.1);

%% ------------------------------------ %%
%% -----------PRINTING PHASE ---------- %%
%% ------------------------------------ %%

% plot all the landmarks in my_initial_condition
figure(1)
xlim([-15,15]);
ylim([-15,15]);
title("initial prevision");
grid on
hold on

%% print my results as initial conditions
for l = 1:length(my_initial_condition)

	plot(my_initial_condition(l).landmark_position(1), my_initial_condition(l).landmark_position(2),"square","markersize",8,"color",'k',"markerfacecolor",'k');
	pause(0.1)

endfor



%% ------------------------------------ %%
%% - PLOT LANDMARKS IN GROUND TRUTH---- %%
%% ------------------------------------ %%


for l = 1:length(landmarks_gt)

	plot(landmarks_gt(l).x_pose, landmarks_gt(l).y_pose,"square","markersize",8,"color",'r',"markerfacecolor",'r');
	pause(0.1)

endfor


%% ------------------------------------ %%
%% ----- EVALUATING INITIAL GUESS ----- %%
%% ------------------------------------ %%

simili = [];

for i=1:length(my_initial_condition)

	id = my_initial_condition(i).id;
	% search it in landmarks_gt
	
	guessx = my_initial_condition(i).landmark_position(1);
	guessy = my_initial_condition(i).landmark_position(2);
	
	for j=1:length(my_initial_condition)
	
		if(id==landmarks_gt(j).landmark_id)
		% we've a correspondence
		
		groundx = landmarks_gt(j).x_pose;
		groundy = landmarks_gt(j).y_pose;
		
		% check if they're similar
		
		diffx = abs(guessx-groundx);
		diffy = abs(guessy-groundy);
		
		if(diffx < 1 && diffy < 1)
			% they're similar !
			
			simili(end+1) = id;
		
		endif
		
		
		endif
	
	endfor


endfor


printf("\nhow many similar %f \n",length(simili));

for i=1:length(simili)

	idsimle = simili(i);

	for l = 1:length(landmarks_gt)
			
		if(landmarks_gt(l).landmark_id == idsimle)
			plot(landmarks_gt(l).x_pose, landmarks_gt(l).y_pose,"square","markersize",8,"color",'g',"markerfacecolor",'g');
			pause(0.1)
		endif
		
	endfor
	
endfor

%% ------------------------------------ %%
%% ------------ END-------------------- %%
%% ------------------------------------ %%
