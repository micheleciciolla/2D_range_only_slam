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

[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');

landmark_lookout = [];
id = [];
count = [];

	function yes = inside(list,elem)

	yes = false;
	for e =1:length(list)

	if list(e) == elem
		yes = true;
	endif
	endfor
	

for s=1:length(observations)
	
	sample = observations(s);
	current_pose = sample.pose_id;

	
	for o=1:length(sample.observation)
	
		landmark_and_range = sample.observation(o);
		landmark_seen = landmark_and_range.landmark_id;
		
		% now you^re seeying this landmark
		% check if is inside the list
		
		if inside(id,landmark_seen)
			count(landmark_seen) = count(landmark_seen) +1.0;
			
		else
			id(end+1) = landmark_seen;
			count(landmark_seen) = 1.0;
		endif
		
		
	endfor


endfor

disp("done");


found = 0;
for i=1:length(id)
	
	% for each id tell me how many you found
	if id(i) > 0
		found = count(id(i));
		printf('\nlandmark %f is found %f times\n', id(i), found);
	endif
endfor


return









function lookout = check_add(lookout,pose,landmark)

% ogni elemento di lookout è un [landmark_id, count, [pose1,pose2,pose3...] ]

% vedi se c'è landmark

	for e=1:length(lookout)
			if lookout(e).landmark_id == landmark
				% update
				lookout(e).count = lookout(e).count +1;
				lookout(e).poses(end+1) = pose;
			endif
			
			% add it
			lookout(end+1).landmark_id = landmark;
			lookout(end).count = 1;
			lookout(end).poses = [pose];
	endfor

end %end function











return 
for j=1:3

	sample = observations(j);
	posa = sample.pose_id;
	
	fprintf('\ncurrent pose n. %f :\n',posa)

	for i=1:length(sample.observation)
	
		landmark_and_range = sample.observation(i)
		fprintf('\n	landmark %f is %f far\n',landmark_and_range.landmark_id, landmark_and_range.range_measure)

	endfor
		
endfor


