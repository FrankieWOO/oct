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
function W = fn_weight_normalised_gaussians(X,c,s2)

%Nc = size(c,2);     % get no. local models
%D = distances(c,X); % compute distances
%W = exp(-(0.5/s2)*D);
%W = W.*repmat(sum(W).^(-1),Nc,1); % normalise W
%W

N  = size(X,2);     % get no. data points
Nc = size(c,2);     % get no. local models
for nc=1:Nc
	d = X-repmat(c(:,nc),1,N); % distances of all data points from this model
	for n=1:N
		W(nc,n) = exp(-0.5*d(:,n)'*diag(1./s2)*d(:,n));
	end
end
W = W.*repmat(sum(W).^(-1),Nc,1); % normalise W
%W

%Nc = size(c,2);     % get no. local models
%D = distances(c,X); % compute distances
%W = exp(-(0.5/s2)*D);
%W = W.*repmat(sum(W).^(-1),Nc,1); % normalise W

