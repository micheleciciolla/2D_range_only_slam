% odometric calibration
clc
clear 
close all

% loading both datasets
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');
[landmarks_ground_truth, poses_ground_truth, transitions_ground_truth, observations_ground_truth] = loadG2o('slam2d_range_only_ground_truth.g2o');

figure()
hold on;
grid minor;

poses = poses_ground_truth;
transitions = transitions_ground_truth;

% plot all poses
for p=3:length(poses)-1

	prevx = poses(p).x;
	prevy = poses(p).y;
	
	prevx2 = poses(p-1).x;
	prevy2 = poses(p-1).y;
	
	odomx = poses(p-1).x + transitions(p).v(1);
	odomy = poses(p-1).y + transitions(p).v(2);
	
	odomx2 = poses(p-2).x + transitions(p-1).v(1);
	odomy2 = poses(p-2).y + transitions(p-1).v(2);
	
	
	pause(.001);
	
	% plot([x1g x2g],[y1g y2g],'linestyle',"-",'marker','o','color','b','linewidth',3);


	plot([prevx prevx2],[prevy prevy2],'linestyle',"-",'marker','.','color','r','linewidth',3);
	plot([odomx odomx2] ,[odomy odomy2],'linestyle',"-",'marker','.','color','b','linewidth',3);

endfor

legend("guess","odometry")

% ------------------------------------------------------------------------------
return 
% ------------------------------------------------------------------------------

figure()
hold on;
grid minor;

% plot all poses
for p=3:length(poses)-1

	prevtheta = poses(p).x;
	prevtheta2 = poses(p-1).x;

	odomtheta = poses(p-1).theta + transitions(p).v(3);
	odomtheta2 = poses(p-2).theta + transitions(p-1).v(3);
	
	pause(.001);
	
	% plot([x1g x2g],[y1g y2g],'linestyle',"-",'marker','o','color','b','linewidth',3);

	plot([p p+1],[ prevtheta2 prevtheta ],'linestyle',"-",'marker','o','color','r','linewidth',3);
	plot([p p+1],[ odomtheta2 odomtheta ],'linestyle',"-",'marker','o','color','b','linewidth',3);


endfor

legend("guess","odometry")