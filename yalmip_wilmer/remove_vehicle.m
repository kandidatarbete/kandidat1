
function [task] = remove_vehicle(idx,task)
task.V(idx) = []; 
task.Nv = task.Nv -1;
end