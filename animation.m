% animation
close all;
% XR_correction,XL_correction
% associations

% concept
% per ogni posa, stampa tutti i landmark che vedi da li
figure();
hold on;
xlim([-15,15]);
ylim([-15,10]);
pause(3);
for o=1:length(observations)
	
	% get the pose_id
	id = observations(o).pose_id - 1100;
	% get the pose associated
	
	[xr,yr] = deal( XR_correction(1,3,id), XR_correction(2,3,id));
	% get all the landmark seen from there, use associations
	
	plot(xr,yr,"^","markersize",15,"color",'k',"markerfacecolor",'k');

	for a=1:length(associations)
		
		if associations(1,a) == id
			% this is a landmark to plot

			plot(XL_correction(1,associations(2,a)), XL_correction(2,associations(2,a)),"p","markersize",15,"color",'k',"markerfacecolor",'b','linestyle','none');
			pause(0.0001);
		endif
	% hold off;
	endfor
	
	plot(xr,yr,"-^","markersize",13,"color",'k',"markerfacecolor",'r');

endfor
legend("robot","landmark");