function addr = cip2addr_single(chan, inst, phas)

	% Applies BSM-159,1
	% Address indexing can be seen in Table-159,3 (TE-159)
	
	% Get binary representation of input
	c = dec2binv(chan, 4);
	i = dec2binv(inst, 7);
	p = dec2binv(phas, 4);
	
	% Create address array
	addr_bv = [ c(3+1), c(2+1), c(1+1), c(0+1), i(5+1),...
		     i(4+1), i(3+1), i(2+1), p(1+1), p(2+1),...
			 i(6+1), p(3+1), i(1+1), p(0+1), i(0+1)];
		 
	% Get decimal address
	addr = binv2dec(addr_bv);
	
end