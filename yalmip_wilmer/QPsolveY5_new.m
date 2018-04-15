function res=QPsolveY5_new(task)
V=task.V; Ns=task.Ns; Nv=task.Nv; ds=task.ds;
co=task.crossingorder(1:Nv);

% scaling factors
St=task.St; Sz=task.Sz; Sdz=task.Sdz; Sddz=task.Sddz; Scost=task.Scost;

%penalties
Wv=task.Wv; Wdv=task.Wdv; Wddv=task.Wddv;
%%
yalmip('clear')
%TODO vref ska inte vara h�rdkodad
vref=50;
vstart=12;
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
for i = 1:Nv
    for j = 1:Ns
        Xhat(j*4-3,i)= t(i,j);
        Xhat(j*4-2,i)= z(i,j);
        Xhat(j*4-1,i)=dz(i,j);
        Xhat(j*4,i)=u(i,j);
    end
end

% scaling factors for control signal u
Su=zeros(3*Nv);
SuSub=eye(3);
SuSub(1,1)=Sz/St;
SuSub(2,2)=Sdz/Sz;
SuSub(3,3)=Sddz/Sdz;
for i=1:Nv
    Su(3*i-2:3*i,3*i-2:3*i)=SuSub;
end
constraints  = [];

Asub = [1 ds 0; 0 1 ds; 0 0 1]; % A matrix for 1 vehicle
% construct generalized A:
for i=1:Nv
    A(3*i-2:3*i,3*i-2:3*i)=Asub;
end

% construct generalized submatrix A coupling both u_k and x_k one vehicle
A_sub_gen = [1 ds 0 0 ; 0 1 ds 0; 0 0 1 Sddz/Sdz*ds; 0 0 0 0];

A_gen = zeros(4*Ns,4*Ns);

%construct global generalized A coupling both u_k and x_k
for i=1:Ns
    A_gen(4*i-3:4*i,4*i-3:4*i)=A_sub_gen;
end
eyesub = zeros(4*Ns,4*Ns);
for i=1:Ns-1
    eyesub(4*i+1:4*i+4,4*i-3:4*i)=eye(4);
end
% longitudinal dynamics in terms of generalized A
A_gen_final = (eye(4*Ns) - eyesub*A_gen);
A_gen_final_2=A_gen_final(5:4*Ns,:);
%constraints=[constraints, A_gen_final_2*Xhat==0];

%Xhat2 = zeros(4*Nv*Ns,1); 
Xhat2=[];
for i = 0:Nv-1
    Xhat2 = [Xhat2; Xhat(:,i+1)]; 
end
disp('done constructing Xhat2'); 

% vehicle j, sample i
tind = @(i,j) 4*i-3 + 4*Ns*(j-1);
zind = @(i,j) 4*i-2 + 4*Ns*(j-1);
dzind = @(i,j) 4*i-1 + 4*Ns*(j-1); 
uind = @(i,j) 4*i + 4*Ns*(j-1);
disp(size(Xhat2));
%disp(dzind(Ns-1,Nv)); 

%Acol = zeros(4*Ns*Nv,4*Ns*Nv); 
Acol = []; 
for i = 1:Ns-1
    for j = 1:Nv         
        cond = zeros(4,4*Ns*Nv);
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
        Acol = [Acol; cond];
        
    end
end
disp('done constructing matrix'); 
disp(size(Acol)); 

constraints=[constraints; Acol*Xhat2 == 0]; 

Aeq2=zeros(4*Ns);
for i=1:3
    Aeq2(i,i)=1;
end

beq2 = zeros(4*Ns,Nv);
for i=1:Nv
    for j = 1:Ns
        % equality constraints
        beq2(1, i) = 0;
        beq2(2,i) = 1/vstart/Sz;
        beq2(3,i) = 0;
        
        % less than constraints
        c1 = -amin*3*vref*Sz/(vref^3)/Sdz;
        c2 = 2*amin/vref^3/Sdz;
        %constraints = [constraints, X(3*i,j) - c1*X(3*i-1,j)  <= c2];
        %equation 1
        %constraints=[constraints, X(3*i-1,:)<=1/V(i).vxmin/Sz];
        
        % greater than constraints
        c3 = -V(i).axmax*3*vref*Sz/(vref^3)/Sdz;
        c4 = V(i).axmax*2/vref^3/Sdz;
        %constraints = [constraints, -X(3*i,j) + c3*X(3*i-1,j) <= c4];
        % equation 2
    end
end
% box constraints
% Xhat \in R^(3*Ns,Nv) -> A \in R^(x, 3*Ns), b \in R^(x, Nv), x = 1
% equation 1
Apa = zeros(1,4*Ns); 
Bepa = zeros(1,4*Ns);
for i = 1:Ns
    for j = 1:Nv
       Apa(4*i-1) = 1;
       Apa(4*i-2) = -c1;
       b1(1,j) = c2;
       
       Bepa(4*i -1) = -1; 
       Bepa(4*i -2) = c3;
       b2(1,j) = c4;
    end
end

%constraints = [constraints, Bepa*Xhat <= b2];
cepa = [Apa;Bepa];
depa = [b1;b2];
[Apa;Bepa]*Xhat == [b1;b2];
constraints = [constraints, cepa*Xhat <= depa]; 


%lowerbound constraints
lb=zeros(4*Ns,Nv);
for i=1:Ns
    for j=1:Nv
        lb(4*i-2,j)=1/V(j).vxmin/Sz;
    end
     
end
for i=1:Ns
    for j=Nv
        constraints=[constraints, Xhat(4*i-2,j)<=lb(4*i-2,j)];
    end
    
end
%upperbound constraints
ub=zeros(4*Ns,Nv);
for i=1:Ns
    for j=1:Nv
        ub(4*i-2,j)=1/V(j).vxmax/Sz;
    end
      
end
for i=1:Ns
    for j=Nv
        constraints=[constraints, Xhat(4*i-2,j)>=ub(4*i-2,j)];
    end  
end
constraints=[constraints, Aeq2*Xhat==beq2];

%Aoc = zeros(4*Ns,4*Ns);
% Xhat \in R^(4*Ns,Nv), Aoc*Xhat = y -> Aoc \in R^(x,4*Ns),x = Nv
for i = 1:Nv-1
    Aoc = zeros(Nv,4*Ns);
    ind1 = 4*V(co(i)).Nze-3; % ind1 \in [365, 405] = [1, 4*Ns]
    ind2 = co(i); % ind2 \in [1,3] = [1 ,Nv]
    ind3 = 4*V(co(i+1)).Nzs-3; % ind3 \in [346 425] = [1, 4*Ns]
    ind4 = co(i+1); % ind4 \in [1,4] = [1, Nv];
    constraints = [constraints, Xhat(ind1,ind2) - Xhat(ind3,ind4) <= 0]; 
end

for i = 1:Nv-1

end 
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
    display('optimization failed for some reason');
end

end