%% ------------------------------------ %%
%% evaluating if convergence is good -- %%
%% ------------------------------------ %%
close all;
similar = [];
threshold = 1;
% -----------------------------------------------------------------------------
 
% -----------------------------------------------------------------------------

figure();
hold on, grid minor;

counter_before=0;
for l=1:length(XL_guess)
	
	[xlg, ylg] = deal (XL_guess(1,l), XL_guess(2,l));
	[xlt, ylt] = deal (XL_true(1,l), XL_true(2,l));
	
	if ( abs(xlg - xlt) < threshold && abs(ylg - ylt) < threshold)
		plot(xlg, ylg,"square","markersize",6,"color",'k',"markerfacecolor",'r');
		plot(xlt, ylt,"square","markersize",6,"color",'k',"markerfacecolor",'b');
		pause(0.001);
		counter_before++;
	endif

endfor
title(int2str(counter_before));
legend("XL guess","XL true");
printf("landmarks XL_guess - XL_true : %i",counter_before);
fflush(stdout);


% -----------------------------------------------------------------------------
 
% -----------------------------------------------------------------------------
figure();
hold on, grid minor;

counter_after = 0;
for l=1:length(XL_guess)
	
	[xl, yl] = deal (XL_correction(1,l), XL_correction(2,l));
	[xlt, ylt] = deal (XL_true(1,l), XL_true(2,l));
	
	if ( abs(xl - xlt) < threshold && abs(yl - ylt) < threshold)
		plot(xlt, ylt,"square","markersize",6,"color",'k',"markerfacecolor",'r');
		plot(xl, yl,"square","markersize",6,"color",'k',"markerfacecolor",'b');
		pause(0.001);
		counter_after++;
	endif

endfor
title(int2str(counter_after));
legend("XL correction","XL true");
printf("landmarks XL_correction - XL_true : %i",counter_after);
fflush(stdout);

% -----------------------------------------------------------------------------
 