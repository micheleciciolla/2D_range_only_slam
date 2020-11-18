 close all;
 ax = figure();
 
 ax(1) = subplot (221);
 set (ax(1), "tag", "1");
 hold on;
	for p=1:length(poses_ground_truth)-1
		
		x1 = poses_ground_truth(p).x;
		y1 = poses_ground_truth(p).y;
		
		x2 = poses_ground_truth(p+1).x;
		y2 = poses_ground_truth(p+1).y;
		
		plot([x1 x2],[y1 y2],'linestyle',"-",'marker','o','color','b','linewidth',2);
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
	
 title ("GROUND TRUTH AND INITIAL GUESS"); 
%------------------------------------------------------------------------------
 ax(2) = subplot (222);
 set (ax(2), "tag", "2");
 
	hold on;
		for p=1:length(poses_ground_truth)-1
			
			x1 = poses_ground_truth(p).x;
			y1 = poses_ground_truth(p).y;
			
			x2 = poses_ground_truth(p+1).x;
			y2 = poses_ground_truth(p+1).y;
			
			plot([x1 x2],[y1 y2],'linestyle',"-",'marker','o','color','b','linewidth',2);
			pause(0.001);

		endfor

		for p=1:length(poses_ground_truth)-1

			[x1g, x2g, y1g, y2g] = deal(XR(1,3,p),XR(1,3,p+1), XR(2,3,p),XR(2,3,p+1));

			plot([x1g x2g],[y1g y2g],'linestyle',"-",'marker','o','color','k','linewidth',2);
			pause(.001);

		endfor
 
 title ("GROUND TRUTH AND CORRECTION");
%------------------------------------------------------------------------------

 ax(3) = subplot (223);
 set (ax(3), "tag", "3");
 hold on;
 
 title ("INITIAL GUESS AND CORRECTION");
 
	for p=1:length(poses_ground_truth)-1

		[x1g, x2g, y1g, y2g] = deal(XR(1,3,p),XR(1,3,p+1), XR(2,3,p),XR(2,3,p+1));

		plot([x1g x2g],[y1g y2g],'linestyle',"-",'marker','o','color','k','linewidth',2);
		pause(.001);

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
%------------------------------------------------------------------------------

