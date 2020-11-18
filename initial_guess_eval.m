% initial guess evaluation

% landmarks_ground_truth are the real landmark positions
% landmarks are the guessed landmark positions

% they're organized with the same order ID


function count = initial_guess_eval(landmarks_ground_truth,landmarks)
	threshold = 0.5;
	count=0;

	for l=1:length(landmarks_ground_truth)
		
		if ( abs(landmarks_ground_truth(l).x_pose - landmarks(l).landmark_position(1)) < threshold ...
				&& abs(landmarks_ground_truth(l).y_pose - landmarks(l).landmark_position(2)) < threshold)
				
				% they're similar
				
				count++;
				
		endif
	endfor
	printf("\nYour initial guess has %i true positions\n",count);
	
endfunction