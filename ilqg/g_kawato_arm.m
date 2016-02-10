% Dynamics function for the Kawato arm model for use with ILQR/G.
%
% in: 
%     x     - state [position; velocity]
%     u     - command [muscle activations]
%     model - model struct
% 
% out:
%     xdot,xdot_x,xdot_u - state change and derivatives
%
function [xdot, xdot_x, xdot_u] = g_kawato_arm ( x, u, model )

xdot = f_kawato_arm ( x, u, model );

% compute xdot_x, xdot_u
if nargout > 1
	% analytical derivatives
	dimU = model.dimU;
	dimQ = model.dimQ;
	dimX = 2*dimQ;

	q   = x(1:2);   % joint angles
	qdot= x(3:4);   % joint velocities
	tau = tau_kawato_arm(q,qdot,u,model); % torque	
	Mq  = Mq_planar_2_link_arm(q,model); invMq=inv(Mq);
	L   = model.L;  % link lengths
	M   = model.M;  % mass
	Lg  = model.Lg; % center of gravity
	A   = model.A;
	lm  = model.lm_l0;
	gk  = model.gk;
	gb  = model.gb;
	K   = diag(km_kawato_arm(u,model));
	B   = diag(bm_kawato_arm(u,model));
	R   = diag(model.gr);

	xdot_x        =  zeros(dimX);
	xdot_x(1:2,:) = [zeros(dimQ) eye(dimQ)];
	dTdx1 = -K*A*[1;0]; dtaudx1 = A'*dTdx1; 
	xdot_x(3:4,1) = invMq*dtaudx1;

	fv=model.viscous_friction;	
	c=M(2)*L(1)*Lg(2)*sin(q(2));
    C=c*[-2*qdot(2),-qdot(2);...
	        qdot(1),      0];
	dMqdx2 = [-2*M(2)*L(1)*Lg(2)*sin(q(2)),-M(2)*L(1)*Lg(2)*sin(q(2));...
		        -M(2)*L(1)*Lg(2)*sin(q(2)),0                        ];
    dinvMqdx2 = -invMq*dMqdx2*invMq;
	dCqdotdx2 = M(2)*L(1)*Lg(2)*cos(q(2))*[-2*qdot(2),-qdot(2); qdot(1),0]*qdot;
	dTdx2 = -K*A*[0;1]; dtaudx2 = A'*dTdx2;
   	xdot_x(3:4,2) = dinvMqdx2*tau - dinvMqdx2*(C*qdot) + invMq*dtaudx2 - invMq*dCqdotdx2;
	xdot_x(3:4,3) = -invMq*A'*B*(A*[1;0]) + invMq*[ 2*c*qdot(2); -2*c*qdot(1)] - [fv;0];
    xdot_x(3:4,4) = -invMq*A'*B*(A*[0;1]) + invMq*[ 2*c*qdot(2) + 2*c*qdot(1); 0] - [0;fv];

	xdot_u        =  zeros(dimX,dimU);
	dKdu1 = diag([gk(1),0,0,0,0,0]); dBdu1 = diag([gb(1),0,0,0,0,0]); dTdu1 = dKdu1*R*u - dKdu1*A*q + dKdu1*lm - dBdu1*A*qdot + K*R*[1;0;0;0;0;0]; xdot_u(3:4,1) = invMq*(A'*dTdu1);
	dKdu2 = diag([0,gk(2),0,0,0,0]); dBdu2 = diag([0,gb(2),0,0,0,0]); dTdu2 = dKdu2*R*u - dKdu2*A*q + dKdu2*lm - dBdu2*A*qdot + K*R*[0;1;0;0;0;0]; xdot_u(3:4,2) = invMq*(A'*dTdu2);
	dKdu3 = diag([0,0,gk(3),0,0,0]); dBdu3 = diag([0,0,gb(3),0,0,0]); dTdu3 = dKdu3*R*u - dKdu3*A*q + dKdu3*lm - dBdu3*A*qdot + K*R*[0;0;1;0;0;0]; xdot_u(3:4,3) = invMq*(A'*dTdu3);
	dKdu4 = diag([0,0,0,gk(4),0,0]); dBdu4 = diag([0,0,0,gb(4),0,0]); dTdu4 = dKdu4*R*u - dKdu4*A*q + dKdu4*lm - dBdu4*A*qdot + K*R*[0;0;0;1;0;0]; xdot_u(3:4,4) = invMq*(A'*dTdu4);
	dKdu5 = diag([0,0,0,0,gk(5),0]); dBdu5 = diag([0,0,0,0,gb(5),0]); dTdu5 = dKdu5*R*u - dKdu5*A*q + dKdu5*lm - dBdu5*A*qdot + K*R*[0;0;0;0;1;0]; xdot_u(3:4,5) = invMq*(A'*dTdu5);
	dKdu6 = diag([0,0,0,0,0,gk(6)]); dBdu6 = diag([0,0,0,0,0,gb(6)]); dTdu6 = dKdu6*R*u - dKdu6*A*q + dKdu6*lm - dBdu6*A*qdot + K*R*[0;0;0;0;0;1]; xdot_u(3:4,6) = invMq*(A'*dTdu6);
end

