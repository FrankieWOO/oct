
function a = get_action_vector_gridworld(u)

switch u
case 1
	a = [ 0; 0];
case 2
	a = [ 0; 1];
case 3
	a = [ 1; 0];
case 4
	a = [ 0;-1];
case 5
	a = [-1; 0];
otherwise
	error('unknown action')
end


