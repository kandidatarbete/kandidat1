function out = fulmpc2(a,s,v, time)
    %FULMPC Summary of this function goes here
    %   Wrapper för optimize task samt interface mot simulink
    % v s och a kolonnvektorer
    
    out=v;
    out(1)=60;
    out(2)=30;
    out(3)=10;
    out(4)=90;
    
   
end

