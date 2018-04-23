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

% 
% Xhat2=[];
% for i = 0:Nv-1
%     Xhat2 = [Xhat2; Xhat(:,i+1)]; 
% end
disp('done constructing Xhat2'); 

% sample i, vehicle j
tind = @(i,j) 4*i-3 + 4*Ns*(j-1);
zind = @(i,j) 4*i-2 + 4*Ns*(j-1);
dzind = @(i,j) 4*i-1 + 4*Ns*(j-1); 
uind = @(i,j) 4*i + 4*Ns*(j-1);
disp(size(Xhat2));
%disp(dzind(Ns-1,Nv)); 

% box constraints
A2eq = []; 
A2eq = sparse(A2eq);
bcol = []; 
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
        bcol = [bcol; [0 0 0 0]']; 
        
    end
end
disp('done constructing matrix'); 
disp(size(A2eq)); 

% box constraints
 
%matris f�r v�nsterled i ekvation f�r startv�rden
Aeq2=zeros(4*Ns);

for i=1:3
    Aeq2(i,i)=1;
end


% 
% %trying to create a matrix A which satisfies AX=B for the initial valuees
% %04-22
% Asub=eye(3);
% A=zeros(3*Nv,4*Ns*Nv);
% j=1;
% i=1;
% while(j<=3*Nv)
%     A(j:j+2,i:i+2)=Asub;
%     i=i+4*Ns;
%     j=j+3;
% end
% B=zeros(3*Nv,1);
% 
% for i=1:Nv
%     B(i)=0;
%     B(i+1)=1/vstart(i)/Sz;
%     B(i+2)=0;
%     
% end
beq1 = zeros(4*Ns,Nv);
Aeq11 = zeros(1,4*Ns); 
Aeq12 = zeros(1,4*Ns);
for i=1:Nv
    c1 = -amin*3*vref*Sz/(vref^3)/Sdz;
    c2 = 2*amin/vref^3/Sdz;
    c3 = -V(i).axmax*3*vref*Sz/(vref^3)/Sdz;
    c4 = V(i).axmax*2/vref^3/Sdz;
    %rhs for initial values
    beq1(1, i) = 0;
    beq1(2,i) = 1/vstart(i)/Sz;
    beq1(3,i) = 0;
    
    for j = 1:Ns
        Aeq11(4*j-1) = 1;
        Aeq11(4*j-2) = -c1;
        b1(1,i) = c2;
        
        Aeq12(4*j -1) = -1;
        Aeq12(4*j -2) = c3;
        b2(1,i) = c4; % todo build this in terms of Xhat2 and append to Acol    
    end
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
    constraints = [constraints, Xhat2(xind1) <= Xhat2(xind2)]; % append this condition to Acol, not Aineq
%     
%     cond = zeros(1,4*Nv*Ns);
%     cond(xind1) = 1;
%     cond(xind2) = -1;
%     Acol = [Acol;cond];
%     bcol = [bcol; 0];
    
end

Aineq_f = [Aeq11;Aeq12];
bineq_f = [b1;b2];


vln = 100000; % very small and large numbers, used for non-constraints
vsn = - vln;
%lowerbound constraints
ub=zeros(4*Ns,Nv);
for i=1:Ns
    for j=1:Nv
        ub(4*i-3,j)= vln; 
        ub(4*i-2,j)=1/V(j).vxmin/Sz;
        ub(4*i-1,j) = vln; 
        ub(4*i,j) = vln; 
    end
     
end
%upperbound constraints
lb=zeros(4*Ns,Nv);
for i=1:Ns
    for j=1:Nv
        lb(4*i-3,j) = vsn; 
        lb(4*i-2,j)=1/V(j).vxmax/Sz;
        lb(4*i-1,j) = vsn; 
        lb(4*i,j) = vsn; 
    end
      
end
constraints = [constraints; Xhat >= lb]; 
constraints = [constraints; Xhat <= ub]; 
constraints = [constraints, Aineq_f*Xhat <= bineq_f]; 
constraints=[constraints, Aeq2*Xhat==beq1];
constraints=[constraints; A2eq*Xhat2 == 0];

cost=[];
cost1=[];
cost2=[];
cost3=[];
for i=1:Nv
    cost1 = [cost1,Wv*vref^3.*sum((X(3*i-1,:)-1/vref).^2)]; % equation 4a
    cost2 = [cost2, Wdv*vref^5.*sum((X(3*i,:).^2))]; % equation 4b
    cost3 = [cost3, Wddv*vref^7.*sum((U(3*i,:).^2))]; % equation 4c
end
cost=[cost,cost1,cost2,cost3]./Scost;
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