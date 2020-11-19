% get_Z
function Z = get_Z()

	% loading both datasets
	[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
	[landmarks_ground_truth, poses_ground_truth, transitions_ground_truth, observations_ground_truth] = loadG2o('slam2d_range_only_ground_truth.g2o');

	Z = zeros(length(transitions),6);
	
	for i=1:length(transitions)
		
		Z(i,:) = [transitions_ground_truth(i).v', transitions(i).v'];
		
	endfor



endfunction