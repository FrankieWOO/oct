% Normalised Gaussian weighting function.
%
%  W = fn_weight_normalised_gaussians(X,c,s2)
%
% in:
%     X  - data matrix (each column represents a data point).
%     c  - Gaussian centres
%     s2 - Gaussian widths
%
% out:
%     W       - weight matrix
%
function W = fn_weight_gaussians(X,c,s2)

N  = size(X,2);     % get no. data points
Nc = size(c,2);     % get no. local models
for nc=1:Nc
	d = X-repmat(c(:,nc),1,N); % distances of all data points from this model
	for n=1:N
		W(nc,n) = exp(-0.5*d(:,n)'*diag(1./s2)*d(:,n));
	end
end

