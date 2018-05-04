function out = fulmpc(a,v,s, entryangle, exitangle, time)
    %FULMPC Summary of this function goes here
    %   Wrapper för optimize task samt interface mot simulink
    % v s och a kolonnvektorer
    persistent task 
%     time = double(time);
       aout = a;
       vout = v;
%     disp(time*10000)
%     disp(time*10000<0.00001)
%     disp(a);
%     disp(v); 
%     disp(s); 
%     
    if time==0 
        task = gen_task(false); 
        task = optimize_task(task) 
        disp('optimerar')
    end
% %    % disp(task.res.a(1,1))
% %     %disp(task.res.t(1,1))
    for i = 1:task.Nv
        temp=griddedInterpolant(task.res.t(2:task.Ns,i),task.res.a(:,i));
        aout(task.Nv-i+1)=temp(time);
    end
    
     for i = 1:task.Nv
        temp=griddedInterpolant(task.res.t(1:task.Ns,i),task.res.v(:,i));
        vout(task.Nv-i+1)=temp(time);
    end
       % disp('hej')
   % disp(aout)
    %aout(1:20)=10;
    out=vout;
    
   
end

