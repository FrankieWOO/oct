function rho = rho_ideal_contact( q, model )

l = model.l;
rho = l.*cos(q);

end

