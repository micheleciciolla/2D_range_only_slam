% guess evaluation

% XL_true is the real landmark positions
% XL is the guessed landmark positions

% they're organized with the same order ID

function count = initial_guess_eval(XL_true,XL_guess)
	threshold = 1.5;
	count=0;

	for l=1:length(XL_true)
		
		%if ( abs(XL_true(1,l) - XL_guess(1,l)) < threshold ...
		%		&& abs(XL_true(2,l) - XL_guess(2,l)) < threshold)
				
		if ( norm(XL_true(:,l) - XL_guess(:,l)) < threshold )
				
				% they're similar
				
				count++;
				
		endif
	endfor
	printf("\nYour guess has %i correct positions\n",count);
	
endfunction