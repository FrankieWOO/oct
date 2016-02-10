
function Yp = predict_local_linear(X,model)

N  = size(X,2);       % get no. data points

% find feature vectors
Phi = model.phi(X);
dimPhi = size(Phi,1);

% find weights
W = model.W(X);
Nc = size(W,1);   % get no. data points, dimensionality

% predict training data
for nc=1:Nc
Yp(:,:,nc)=((repmat(W(nc,:),dimPhi,1).*Phi)'*model.w(:,:,nc))';
%Yp(:,:,nc) = sum(repmat(model.w(:,:,nc),1,N).*Phi).*W(nc,:);
end
Yp = sum(Yp,3)./repmat(sum(W,1),size(Yp,1),1);


%function Yp = predict_local_linear(X,model)
%
%N  = size(X,2);       % get no. data points
%
%% find feature vectors
%Phi = model.phi(X);
%
%% find weights
%W = model.W(X);
%Nc = size(W,1);   % get no. data points, dimensionality
%
%% predict training data
%for nc=1:Nc
%Yp(nc,:) = sum(repmat(model.w(:,nc),1,N).*Phi).*W(nc,:);
%end
%Yp = sum(Yp,1)./sum(W,1);



