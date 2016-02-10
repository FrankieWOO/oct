% Function for computing the root mean sqaured temporal difference error on a set of samples of a value function
%
%    [rmstde d] = get_tde(Q,Qn,j,gamma)
%
% in:
%     Q          - state-action values
%     Qn         - state-action values for next step
%     j          - cost/reward values
%     gamma      - discount factor
%
% out:
%     rmstde     - root mean squared temporal difference error
%     d          - TD error vector (each element gives the TD error for that data point)
%     mtde       - mean absolute TD error over points
%
function [rmstde d mtde] = get_tde(Q,Qn,j,gamma)

N     = size(j,2);         % get no. data points
TQ    = j + gamma*Qn;      % T[Q], where T is the Bellman operator
d     = (Q - TQ)';         % TD error vector
rmstde= sqrt(sum(d.^2)/N); % root mean squared TDE 
absd  = abs(d);            %
mtde  = mean(absd);        %
%nmtde = mtde/std(Q,0,2);   %

%tde   = sqrt(d'*d);   % length of TD error vector
%rmstde= tde/sqrt(N);  % root mean squared TDE (equivalent to rmstde = sqrt(sum(d.^2)/N))
