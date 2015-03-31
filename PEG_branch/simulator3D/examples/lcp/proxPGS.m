function [lcpSoln, err] = proxPGS(MM, qq, lcpSoln0)
	convg = 0.05;
	relax = 1e-3;
	lambda = 0.3;
	itermax = 100;
	err = 100;
	threshold = 1e-10;
	sz = size(lcpSoln0,1);
	for i = 1:itermax
		for j = 1 : sz
			lcpSoln0(j) = max(0, lcpSoln0(j)-convg*(MM(j,:)*lcpSoln0+qq(j) + relax*lcpSoln0(j)));
		end
		pn_err   = max(abs(kanzow(lcpSoln0(1:7), MM(1:7,:)*lcpSoln0+qq(1:7), lambda)));
		alpha_err = max(abs(kanzow(lcpSoln0(8:56), MM(8:56,:)*lcpSoln0+qq(8:56), lambda)));
		s_err     = max(abs(kanzow(lcpSoln0(57:63), MM(57:63,:)*lcpSoln0+qq(57:63), lambda)));
		err = pn_err + alpha_err + s_err;
		if err < threshold
			break;
		end
	end
	lcpSoln = lcpSoln0;
end
