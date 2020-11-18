% run this AFTER main.m
close all
% ----------------------------------------------------

figure()
hold on, title("CHI STATS"), grid minor;
plot(chi_stats)

% ----------------------------------------------------
figure();
hold on;
title("RESULTING MAP");

for l=1:length(landmarks_ground_truth)
	plot(landmarks_ground_truth(l).x_pose, landmarks_ground_truth(l).y_pose,"square","markersize",7,"color",'k',"markerfacecolor",'c');
	plot(XL(1,l),XL(2,l), "square","markersize",7,"color",'k',"markerfacecolor",'r');
endfor
pause(.01);

%% ------------------------------------ %%
%% ----------- PLOT GROUND TRUTH------- %%
%% ------------------------------------ %%

figure();
hold on;
title("GROUND TRUTH");
grid minor;

for l=1:length(landmarks_ground_truth)
	
	plot(landmarks_ground_truth(l).x_pose, landmarks_ground_truth(l).y_pose,"square","markersize",6,"color",'k',"markerfacecolor",'c');
	pause(0.001);

endfor

for p=1:length(poses_ground_truth)-1
	
	x1 = poses_ground_truth(p).x;
	y1 = poses_ground_truth(p).y;
	
	x2 = poses_ground_truth(p+1).x;
	y2 = poses_ground_truth(p+1).y;
	
	plot([x1 x2],[y1 y2],'linestyle',"-",'marker','o','color','b','linewidth',2);
	pause(0.001);

endfor

% ----------------------------------------------------

%% ------------------------------------ %%
%% ----------- PLOT INITIAL GUESS ----- %%
%% ------------------------------------ %%

figure();
hold on;
title("BEFORE AND AFTER CORRECTION");
grid minor;

for l=1:length(landmarks)
	
	plot(landmarks(l).landmark_position(1), landmarks(l).landmark_position(2),"square","markersize",6,"color",'k',"markerfacecolor",'m');
	pause(0.001);

endfor

% poses
for p=1:length(poses)-1
	
	x1 = poses(p).x;
	y1 = poses(p).y;
	
	x2 = poses(p+1).x;
	y2 = poses(p+1).y;
	
	plot([x1 x2],[y1 y2],'linestyle',"-",'marker','o','color','r','linewidth',2);
	pause(0.001);

endfor

%% ------------------------------------ %%
%% ----------- PLOT ICP CORRECTION ---- %%
%% ------------------------------------ %%

for p=1:length(poses_ground_truth)-1
	
	[x1g, x2g, y1g, y2g] = deal(XR(1,3,p),XR(1,3,p+1), XR(2,3,p),XR(2,3,p+1));

	plot([x1g x2g],[y1g y2g],'linestyle',"-",'marker','.','color','k','linewidth',2);
	pause(.001);

endfor