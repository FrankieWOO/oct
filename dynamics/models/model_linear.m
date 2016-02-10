% Simple linear model, with two state dimensions, one control dimension.
function model = model_linear

model.dimQ = 1; %
model.dimU = 1; %

model.A = [ 0  1 ; ...
            0  0 ];
model.B = [ 0 ; ...
            1 ];
