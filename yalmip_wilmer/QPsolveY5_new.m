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
%A_sub_gen = [1 ds 0 Sz/St*ds ; 0 1 ds Sdz/Sz*ds; 0 0 1 Sddz/Sdz*ds; 0 0 0 0];
A_sub_gen = [1 ds 0 0 ; 0 1 ds 0; 0 0 1 Sddz/Sdz*ds; 0 0 0 0];
A_sub_gen2 = [0 ds 0 0 ; 0 0 ds 0; 0 0 1 Sddz/Sdz*ds; 0 0 0 0];

A_gen = zeros(4*Ns,4*Ns);
A_gen2 = zeros(4*Ns,4*Ns);
for i = 1:Nv
    A_gen2(4*i-3:4*i,4*i-3:4*i)=A_sub_gen;
    A_gen3(4*i-3:4*i,4*i-3:4*i)=A_sub_gen2; % use this one
end
%construct global generalized A coupling both u_k and x_k
for i=1:Ns
    A_gen(4*i-3:4*i,4*i-3:4*i)=A_sub_gen;
end
A_gen2 = A_gen;
eyesub = zeros(4*Ns,4*Ns);
for i=1:Ns-1
    A_gen2(4*i+1:4*i+4,4*i-3:4*i)=-eye(4);
    eyesub(4*i+1:4*i+4,4*i-3:4*i)=eye(4);
end
% longitudinal dynamics in terms of generalized A
A_gen_final = (eye(4*Ns) - eyesub*A_gen);
A_gen_final_2=A_gen_final(5:4*Ns,:);
constraints=[constraints, A_gen_final_2*Xhat==0];

Aeq2=zeros(4*Ns);
for i=1:3
    Aeq2(i,i)=1;
end


beq2 = zeros(4*Ns,Nv);
for i=1:Nv
    % equality constraints
    beq2(1, i) = 0;
    beq2(2,i) = 1/vstart/Sz;
    beq2(3,i) = 0;
    
    % less than constraints
    c1 = -amin*3*vref*Sz/(vref^3)/Sdz; 
    c2 = 2*amin/vref^3/Sdz; 
    %constraints=[constraints, X(3*i,:)<=-amin*(3*vref*X(3*i-1,:)*Sz - 2)./vref.^3/Sdz];
    %constraints = [constraints, X(3*i,:) <= c1*X(3*i-1,:)  + c2]; 
    constraints = [constraints, X(3*i,:) - c1*X(3*i-1,:) - c2 <= 0]; 
    %constraints=[constraints, X(3*i-1,:)<=1/V(i).vxmin/Sz];
    
    % greater than constraints
    c3 = -V(i).axmax*3*vref*Sz/(vref^3)/Sdz;
    c4 = V(i).axmax*2/vref^3/Sdz; 
    %constraints=[constraints, X(3*i,:) >= -V(i).axmax*(3*vref*X(3*i-1,:)*Sz - 2)./vref^3/Sdz];
    %constraints = [constraints,X(3*i,:) >= c3*X(3*i-1,:) - c4];
    constraints = [constraints, X(3*i,:) - c3*X(3*i-1,:) + c4 >= 0]; 
    %constraints=[constraints, X(3*i-1,:)>= 1/V(i).vxmax/Sz];
    
end


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
%attempt at writing critical constraint onf form A*X<=0,
% A1=zeros(Nv,4*Ns);
% A2=zeros(Nv,4*Ns);
% for i=1:Nv
%     A1(i,4*V(i).Nze-3)=1;
%     A2(i,4*V(i).Nze-3)=1;
% end
% for j=1:Ns-1
%     constraints=[constraints, (A*X)()]
% end

    for i = 1:Nv-1
        %constraints = [constraints,
         %   X(3*co(i)-2,V(co(i)).Nze)-X(3*co(i+1)-2,V(co(i+1)).Nzs) <=0 ];
        
        constraints = [constraints, Xhat(4*V(co(i)).Nze-3,co(i))-Xhat(4*V(co(i+1)).Nzs-3,co(i+1))<=0];
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