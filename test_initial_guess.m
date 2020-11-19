close all
clear
clc
pause(0.01);

addpath '../'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'
addpath './getIDs_and_index.m'
addpath './get_formatted_index.m'



%% ------------------------------------ %%
%% ----------- START ------------------ %%
%% ------------------------------------ %%

% loading both datasets
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
[landmarks_ground_truth, poses_ground_truth, transitions_ground_truth, observations_ground_truth] = loadG2o('slam2d_range_only_ground_truth.g2o');

% testing algo on ground truth
observations = observations_ground_truth;
poses = poses_ground_truth;

%% ------------------------------------ %%
%% ----------- INITIAL GUESS ---------- %%
%% ------------------------------------ %%

%% here i'm getting a LIST of HOW MANY landarmsks and their ID
available_landmarks = zeros(length(landmarks_ground_truth),1);
for l=1:length(landmarks_ground_truth)
	available_landmarks(l) = landmarks_ground_truth(l).landmark_id;
endfor

%% get an INITIAL GUESS of the landmarks given their id, poses and obs from init_guess_dataset
% FIELDS: 
% 	landmarks(i).id
% 	landmarks(i).landmark_position(1) and landmarks(i).landmark_position(2)
landmarks = get_initial_guess(available_landmarks, poses, observations);
% evaluate initial guess wrt landmarks_ground_truth
eval = initial_guess_eval(landmarks_ground_truth,landmarks);
fflush(stdout);

%% ------------------------------------ %%
%% ----------- FUNCTION --------------- %%
%% ------------------------------------ %%

% inputs
% 	available_landmarks is a lsit of all landmaks id
% 	poses are all the poses
% 	observations are all the observations

function my_initial_condition = get_initial_guess(available_landmarks, poses, observations)

	my_initial_condition = [];

	printf('Dataset processed %d%% \n',0);
	for l=1:length(available_landmarks)

		if mod( round(l/length(available_landmarks)*100), 33)==0.0
			printf('Dataset processed %d%% \n',[round(l/length(available_landmarks)*100)]);
			fflush(stdout);
		endif
		
		% now i'm optimizing this landmark id
		searched_landmark = available_landmarks(l);
		
		% retrieve landmark info from the dataset 
		% you get landmark_position, range, id, how many times you got it
		landmark_info = find_landmark_references(searched_landmark,poses,observations);
		% start optimization process for this landmark based on the info you have
		% landmark_pos = converge_landmark_pos(landmark_info);
		landmark_pos = triangulate_landmark_pos(landmark_info);
		
		% update initial_condition
		my_initial_condition(end+1).id = searched_landmark;
		my_initial_condition(end).landmark_position = landmark_pos;

	endfor
	printf('Dataset processed %d%% \n',100);
	fflush(stdout);
	pause(0.1);


end

%% ----------------------------------------- %%
%%  GETTING INFO FROM THE SEARCHED LANDMRK - %%
%% ----------------------------------------- %%
function id_pose_range = find_landmark_references(searched_landmark,poses,observations)
	
	% THESE ARE THE INFOS I WILL SEND BACK FOR EACH SEARCHED_LMK
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
		% do local otpmization
			delta_epsi = pinv(regression(i,:))*error(i);
			epsi = epsi + delta_epsi;
		endif
		
		
	endfor

	fflush(stdout);
	% optmizing one time
	% delta_epsi = pinv(regression)*((error));
	% land_position = epsi + delta_epsi;
	
	% my output
	land_position = epsi;


end

% This is another function to triangulate its position
% input is a vector of information about each landmark
% info(i) has pose.x, pose.y, range measure available from pose i

% this function has better results than converge_landmark_pos 
function landmark_position = triangulate_landmark_pos(info)
	
	num_points = length(info);
	last_x = info(num_points).pose.x;
	last_y = info(num_points).pose.y;
	r_last = info(num_points).range;

	A = zeros(num_points-1,2);
	b = zeros(num_points-1,1);
	
	for(i=1:num_points-1)
		xi = info(i).pose.x;
		yi = info(i).pose.y;
		ri = info(i).range;

		A(i,1) = xi - last_x;
		A(i,2) = yi - last_y;
		b(i) = xi^2-last_x^2 + yi^2-last_y^2 + r_last^2-ri^2;
	end
	b = 0.5*b;
	damp_pinv = inv(A'*A+eye(2)*0.001)*A';

	landmark_position = damp_pinv*b;
end