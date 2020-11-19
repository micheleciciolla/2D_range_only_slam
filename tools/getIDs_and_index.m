% this is a function that return all the real ids of the available landmarks
% and a sequential index used in the iCP

% example
% | 1 2 7 90 |
% | 1 2 3 4  |
function allIDs = getIDs_and_index(observations)

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

					
endfunction 


