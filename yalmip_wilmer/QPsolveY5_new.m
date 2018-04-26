function res=QPsolveY5_new(task)
V=task.V; 
Ns=task.Ns;
Nv=task.Nv; 
ds=task.ds;
co=[1:20];

% scaling factors
St=task.St; Sz=task.Sz; Sdz=task.Sdz; Sddz=task.Sddz; Scost=task.Scost;

%penalties
Wv=task.Wv; Wdv=task.Wdv; Wddv=task.Wddv;
%%
yalmip('clear')
vref=50;
vstart=12*ones(Nv,1);
astart=0*ones(Nv,1);
sstart=0*ones(Nv,1);
amin=-5;
%bromstid
deltat=20;
k=1:Ns-1;
t = sdpvar(Nv,Ns,'full');
z= sdpvar(Nv,Ns,'full');
dz=sdpvar(Nv,Ns,'full');
u=sdpvar(Nv,Ns,'full');

for i=1:Nv
    X(3*i-2,:)=t(i,:);
    X(3*i-1,:)=z(i,:);
    X(3*i,:)=dz(i,:);
end

% control signal for acceleration
for i=1:Nv
    U(3*i,:)=u(i,:);
end
Xhat=sdpvar(4*Ns,Nv);
for i = 1:Nv
    for j = 1:Ns
        Xhat(j*4-3,i)= t(i,j);
        Xhat(j*4-2,i)= z(i,j);
        Xhat(j*4-1,i)=dz(i,j);
        Xhat(j*4,i)=u(i,j);
    end
end
constraints  = [];

 Xhat2 = sdpvar(4*Nv*Ns,1);
 for i=1:Nv
     Xhat2((4*Ns*(i-1))+1:(4*Ns*(i)),1)=Xhat(:,i);
     
 end

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
beq1 = zeros(4*Ns,Nv);


for i=1:3
    Aeq2(i,i)=1;
end



for i = 1:Ns-1
    for j = 1:Nv         
        cond = sparse(4,4*Ns*Nv);
        cond(1,tind(i+1,j)) = 1;
        cond(1,tind(i,j)) = -1;
        cond(1,zind(i,j)) = - ds;
        %Acol = [Acol; cond];
        
        %cond = zeros(1,4*Ns*Nv);
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
   %b2eq = [b2eq; [0 0 0]']; 
end

for i=1:Nv
    %rhs for initial values
    beq1(1, i) = 0;
    beq1(2,i) = 1/vstart(i)/Sz;
    beq1(3,i) = 0;
end

for i = 1:Nv-1
    Aoc = zeros(Nv,4*Ns);
    ind1 = 4*V(co(i)).Nze-3;
    ind2 = co(i);
    ind3 = 4*V(co(i+1)).Nzs-3;
    ind4 = co(i+1);
    %constraints = [constraints, Xhat(ind1,ind2) - Xhat(ind3,ind4) <= 0];
    
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

constraints=[constraints; A2eq*Xhat2 == b2eq];
constraints = [constraints; A2ineq*Xhat2 == b2ineq]; 
constraints = [constraints; lb2 <= Xhat2];
constraints = [constraints; Xhat2 <= ub2]; 

% % sample i, vehicle j
% tind = @(i,j) 4*i-3 + 4*Ns*(j-1);
% zind = @(i,j) 4*i-2 + 4*Ns*(j-1);
% dzind = @(i,j) 4*i-1 + 4*Ns*(j-1); 
% uind = @(i,j) 4*i + 4*Ns*(j-1);

% construct cost function
H = sparse(4*Ns*Nv, 1);
f = sparse(4*Ns*Nv,1); 
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
%H = 0.5*H + 0.5*H'; % symmetrization
cost12 = Xhat2'*H*Xhat2 + f'*Xhat2; 

cost=[];
cost1=[];
cost2=[];
cost3=[];

for i=1:Nv
   % cost1 = [cost1,Wv*vref^3.*sum((X(3*i-1,:)-1/vref).^2)]; % equation 4a
   % cost2 = [cost2, Wdv*vref^5.*sum((X(3*i,:).^2))]; % equation 4b
    %cost3 = [cost3, Wddv*vref^7.*sum((U(3*i,:).^2))]; % equation 4c
end
cost=[cost,cost1,cost2,cost3, cost12]./Scost;
%options     = sdpsettings('verbose',0,'solver','ecos','debug', 1);
options     = sdpsettings('verbose',0,'debug', 1);
sol         = optimize(constraints, sum(cost), options);

%%
if sol.problem == 0
    res.status='Solved';
    res.time=sol.solvertime;
    res.cost=sum(value(cost));
    res.v=[];
    res.t=[];
    for i=1:Nv
        res.v=[res.v; 1./value(X(3*i-1,:))/Sz];
        res.t=[res.t; value(X(3*i-2,:))*St];
    end
    res.v=res.v';
    res.t=res.t';
    %res.v=1./value(z(:,1:Ns))'/Sz;
else
    res.status=sol.info;
    display(sol.info);
    display('optimization failed');
end

end