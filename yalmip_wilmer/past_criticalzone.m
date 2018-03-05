function [past]=past_criticalzone(task, i)   
    for j=1:task.Nv
        if(task.res.t(i,j)*task.res.v(i,j)/task.ds>=task.V(j).Nze)
            past(j)=true;
        else
            past(j)=false;
        end
    end
end