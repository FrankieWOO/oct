% Function for plotting solutions to the finite horizon LQ problem from the demo scripts
%
% in: 
%     figNo - figure number
%     t     - time
%     x     - state (trajectory)
%     u     - commands
%     L     - gains
%     xh    - estimated state
%
function h = plot_lq_fh(figNo,figName,t,x,u,L,xh,colour)

if nargin<8
	colour='b';
end

spr=2; % no. subplot rows
spc=4; % no. subplot columns
h=nan(9,1);

figure(figNo),set(figNo,'Name',figName),%clf
subplot(spr,spc,[1 2 5 6]),hold on,box on,xlabel('x_1'),ylabel('x_2'),h(1)=plot(x (1,:)   ,x (2,:)     ,'Color',colour); h(2)=plot(x (1,end),x (2,end),'.','Color',colour); 
                                                      if ~isempty(xh),h(3)=plot(xh(1,:)   ,xh(2,:),'--','Color',colour); h(4)=plot(xh(1,end),xh(2,end),'o','Color',colour);end
subplot(spr,spc, 3       ),hold on,box on,xlabel('t'  ),ylabel('x_1'),h(4)=plot(t         ,x (1,:)     ,'Color',colour);
                                                      if ~isempty(xh),h(5)=plot(t         ,xh(1,:),'--','Color',colour);end
subplot(spr,spc, 7       ),hold on,box on,xlabel('t'  ),ylabel('x_2'),h(6)=plot(t         ,x (2,:)     ,'Color',colour);
                                                      if ~isempty(xh),h(7)=plot(t         ,xh(2,:),'--','Color',colour);end
if ~isempty(u),
subplot(spr,spc, 4       ),hold on,box on,xlabel('t'  ),ylabel('u'  ),h(8)=plot(t(1:end-1),u           ,'Color',colour);end
if ~isempty(L),
subplot(spr,spc, 8       ),hold on,box on,xlabel('t'  ),ylabel('L'  ),     plot(t(1:end-1),squeeze(L)' ,'Color',colour);end


