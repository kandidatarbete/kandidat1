function res=QPsolveY5_new2(task,astart,vstart,ss)
V=task.V; 
Ns=task.Ns;
Nv=task.Nv; 
ds=task.ds;
co=task.crossorder;

% scaling factors
St=task.St; Sz=task.Sz; Sdz=task.Sdz; Sddz=task.Sddz; Scost=task.Scost;

%penalties
Wv=task.Wv; Wdv=task.Wdv; Wddv=task.Wddv;
%%
yalmip('clear')
vref=50;
%vstart=12*ones(Nv,1);
%astart=0*ones(Nv,1);
%sstart=0*ones(Nv,1);
amin=-5;
%bromstid


disp('done constructing Xhat2'); 

% sample i, vehicle j
tind = @(i,j) 4*i-3 + 4*Ns*(j-1);
zind = @(i,j) 4*i-2 + 4*Ns*(j-1);
dzind = @(i,j) 4*i-1 + 4*Ns*(j-1); 
uind = @(i,j) 4*i + 4*Ns*(j-1);

% box constraints
A2eq = []; 
A2eq = sparse(A2eq);
b2eq = []; 

A2ineq = []; 
A2ineq = sparse(A2ineq);
b2ineq = []; 

ub2 = zeros(4*Ns*Nv,1); 
lb2 = zeros(4*Ns*Nv,1); 

%matris f�r v�nsterled i ekvation f�r startv�rden
Aeq2=zeros(4*Ns);

for i=1:3
    Aeq2(i,i)=1;
end

for i = 1:Ns-1
    for j = 1:Nv         
        cond = zeros(4,4*Ns*Nv);
        cond(1,tind(i+1,j)) = 1;
        cond(1,tind(i,j)) = -1;
        cond(1,zind(i,j)) = - ds;
       
        cond(2,zind(i+1,j)) = 1;
        cond(2,zind(i,j)) = -1;
        cond(2,dzind(i,j)) = - ds;
        %Acol = [Acol; cond];
        
        %cond = zeros(1,4*Ns*Nv);
        cond(3,dzind(i+1,j)) = 1;
        cond(3,dzind(i,j)) = -1;
        cond(3,uind(i,j)) = - Sddz/Sdz*ds;
        
        cond(4,:) = zeros(1,4*Ns*Nv); 
        A2eq = [A2eq; cond];
        b2eq = [b2eq; [0 0 0 0]']; 
        
    end
end
% initial values
for i = 1:Nv
   cond = zeros(3,4*Ns*Nv);
   cond(1,tind(1,i)) = 1; 
   cond(2,zind(1,i)) = 1;
   cond(3,dzind(1,i)) = 1;
   A2eq = [A2eq;cond]; 
   b2eq = [b2eq; [0 1/vstart(i)/Sz 0]'];  
end

for i = 1:Nv-1
    
    sample1 = V(co(i)).Nze;
    vehicle1 = co(i);
    xind1 = tind(sample1,vehicle1);
    
    sample2 = V(co(i+1)).Nzs;
    vehicle2 = co(i+1);
    xind2=tind(sample2,vehicle2);
    %constraints = [constraints, Xhat2(xind1) <= Xhat2(xind2)]; 
    
    cond = zeros(1,4*Nv*Ns);
    cond(xind1) = 1;
    cond(xind2) = -1;
    A2ineq = [A2ineq;cond];
    b2ineq = [b2ineq; 0];
    
end

vln = 100000; % very small and large numbers, used for "non-constraints"
vsn = - vln;


for i = 1:Ns
   for j = 1:Nv
       lb2(tind(i,j)) = vsn; 
       lb2(zind(i,j)) = 1/V(j).vxmax/Sz; 
       lb2(dzind(i,j)) = vsn; 
       lb2(uind(i,j)) = vsn; 
   end
end


for i = 1:Ns
   for j = 1:Nv
       ub2(tind(i,j)) = vln; 
       ub2(zind(i,j)) = 1/V(j).vxmin/Sz; 
       ub2(dzind(i,j)) = vln; 
       ub2(uind(i,j)) = vln; 
   end
end


%constraints = [constraints; Xhat >= lb]; 
%constraints = [constraints; Xhat <= ub]; 
%constraints = [constraints, Aineq_f*Xhat <= bineq_f]; 
%constraints=[constraints, Aeq2*Xhat==beq1]; 

%constraints=[constraints; A2eq*Xhat2 == b2eq];
%constraints = [constraints; A2ineq*Xhat2 == b2ineq]; 
%constraints = [constraints; lb2 <= Xhat2];
%constraints = [constraints; Xhat2 <= ub2]; 

% construct cost function
H = zeros(4*Ns*Nv, 1);
f = zeros(4*Ns*Nv,1); 
for i= 1:Ns
   for j = 1:Nv
      % first cost function
      H(zind(i,j)) = Wv*vref.^3;
      f(zind(i,j),1) = -2*1/vref; 
      
      % second
      H(dzind(i,j)) = Wdv*vref^5; 
      f(zind(i,j),1) = 0; 
      
      % third 
      H(uind(i,j)) = Wddv*vref^7; 
      f(uind(i,j)) = 0; 
   end
end
H=diag(H);
H=sparse(H);

XhatOptimized=quadprog(H,f,A2ineq,b2ineq,A2eq,b2eq,lb2,ub2);
cost12 = XhatOptimized'*H*XhatOptimized + f'*XhatOptimized;
sol         = struct;
sol.problem=0;
%%
if sol.problem == 0
    res.status='Solved';
    
    res.cost=sum(cost12);
    res.v2=[];
    res.t2=[];
    for i=1:Nv
        for j=1:Ns
        
            res.v2(j,i)=1./XhatOptimized(zind(j,i),1)/Sz;
            res.t2(j,i)=XhatOptimized(tind(j,i),1)*St;
        end
    end
    res.v=res.v2;
    res.t=res.t2;
    
    for i=1:Ns-1
        for j=1:Nv
            res.a(j,i)=(res.v(i+1,j)-res.v(i,j))/(res.t(i+1,j)-res.t(i,j));
        end
    end
    res.a=res.a';
    %res.GI=struct;
    GI = cell(Nv,1);
    for i=1:Nv
     GI{i}=griddedInterpolant(res.t(2:Ns,i),res.a(:,i));
      
    end
    
    
    
else
    res.status=sol.info;
    display(sol.info);
    display('optimization failed');
end

end