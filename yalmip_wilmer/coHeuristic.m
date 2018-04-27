function co=coHeuristic(s)
    co=zeros(length(s),1);
       for i=1:length(s)
           s(i)=abs(s(i));
       end
       [V I]=sort(s);
     co=I; 
        
    
end