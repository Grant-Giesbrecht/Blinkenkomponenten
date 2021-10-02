function addr = cip2addr(chan, inst, phas)

	% Get max input length
	len = max([length(chan), length(inst), length(phas)]);

	% Check lengths, make sure all are same
	if length(chan) == 1
		chan = repmat(chan, 1, len);
	elseif length(chan) ~= len
		warning("Channel is wrong length!");
		return;
	end
	if length(inst) == 1
		inst = repmat(inst, 1, len);
	elseif length(inst) ~= len
		warning("Instruction is wrong length!");
		return;
	end
	if length(phas) == 1
		phas = repmat(phas, 1, len);
	elseif length(phas) ~= len
		warning("Phase is wrong length!");
		return;
	end
	
	% Find address for each point
	for idx = 1:len
		addr(idx) = cip2addr_single(chan(idx), inst(idx), phas(idx));
	end
	
	
end