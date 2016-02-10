function Fx = Fx_ideal_contact( q, qdot, model )

l    = model.l;
kb   = model.kb;
bb   = model.bb;
rho0 = model.rho0;

 Fx = kb*(l.*cos(q)-rho0) + bb.*qdot;

end

