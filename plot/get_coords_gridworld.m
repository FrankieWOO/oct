function r = get_coords_gridworld(x)

[X,Y]=meshgrid(linspace(0.5,3.5,4),linspace(0.5,3.5,4));
R = [X(:),Y(:)]';
r = R(:,x);

