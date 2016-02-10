function rdotx = rdotx_ideal_contact( q, qdot, model )

l = model.l;

rdotx = -l.*sin(q).*qdot;

end

