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
	
	starting_xl = starting_xr + starting_range;
	starting_yl = starting_yr + starting_range;

	epsi = [starting_xl; starting_yl];
	
	% initial guess
	% plot(epsi(1),epsi(2),'square',"markersize",7,"color",'b',"markerfacecolor",'b');


	for i=1:samples_size
		
		xl = epsi(1);
		yl = epsi(2);
		
		xr = info(i).pose.x;
		yr = info(i).pose.y;
		
		measurement = info(i).range;
		
		current_r_square = xl^2 + xr^2 - 2*xl*xr + yl^2 + yr^2 - 2*yl*yr;
		% vedi se cambiare metodo o cambiare matrice
		regression = [2*epsi(1) - 2*xr, 2*epsi(2) - 2*yr];
			
		error =  measurement^2 - (current_r_square);

		delta_epsi = pinv(regression)*((error));
		epsi = epsi + delta_epsi;
		
		% plot(epsi(1),epsi(2),'square',"markersize",7,"color",'k',"markerfacecolor",'k');
		%fflush(stdout);
		%pause(0.2);

	endfor
	
	land_position = epsi;

end



disp("------------------");
disp("start");
disp("------------------");

my_initial_condition = [];

for l=1:length(landmarks_gt)

	printf('completed %i perc \n',[l/length(landmarks_gt)*100]);
	fflush(stdout);
	
	searched_landmark = landmarks_gt(l).landmark_id;
	
	landmark_info = find_landmark_references(searched_landmark,poses,observations);
	landmark_pos = converge_landmark_pos(landmark_info);
	
	% update initial_condition
	my_initial_condition(end+1).id = searched_landmark;
	my_initial_condition(end).landmark_position = landmark_pos;

endfor

% plot all the landmarks in my_initial_condition
figure(1)
xlim([-15,15]);
ylim([-15,15]);
title("initial prevision");
grid on
hold on

for l = 1:length(my_initial_condition)

	plot(my_initial_condition(l).landmark_position(1), my_initial_condition(l).landmark_position(2),"square","markersize",8,"color",'k',"markerfacecolor",'k');
	pause(0.1)

endfor


% plot all the landmarks in landmarks_ground_truth

%figure(2)
%xlim([-15,15]);
%ylim([-15,15]);
%title("ground truth");
%hold on
%grid on

for l = 1:length(landmarks_gt)

	plot(landmarks_gt(l).x_pose, landmarks_gt(l).y_pose,"square","markersize",8,"color",'r',"markerfacecolor",'r');
	pause(0.1)

endfor


% evaluating if convergence is good

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
		
		if(diffx < 0.5 && diffy < 0.5)
			% they're similar
			
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






return

% i know real positions of landmark 19 from ground truth
real_pos_19 = [-0.297143 ; 4.91945];
real_pose_1 = [14.0658 ;-8.37047];

real_pos = real_pos_19;
% searched_landmark = 19.0;
searched_landmark = 1.0;

figure()
hold on
plot(real_pos(1),real_pos(2),'square',"markersize",7,"color",'r',"markerfacecolor",'r');

landmark_info = find_landmark_references(searched_landmark,poses,observations);
landmark_pos = converge_landmark_pos(landmark_info);

fprintf("error on x %f\nerror on y %f ",[landmark_pos(1) - real_pos(1), landmark_pos(2) - real_pos(2)]);

