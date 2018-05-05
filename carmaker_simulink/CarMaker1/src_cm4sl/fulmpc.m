function res = fulmpc(a,v,s, entryangle, exitangle, time)
    %FULMPC Summary of this function goes here
    %   Wrapper för optimize task samt interface mot simulink
    % v s och a kolonnvektorer
    

    %task=gen_task2(a,v,s,entryangle,exitangle,false);
    %task = optimize_task(task);
   task=gen_task(false);
   
    res = ones(1,20);
end

