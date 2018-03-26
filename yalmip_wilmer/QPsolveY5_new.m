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

for i = 1:Nv
    X2(4*i-3,:)=t(i,:);
    X2(4*i-2,:)=z(i,:);
    X2(4*i-1,:)=dz(i,:);
    X2(4*i,:) = u(i,:); 
end


%X=sdpvar(3*Nv,Ns,'full');

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

A_gen = zeros(4*Ns,4*Ns); 
A_gen2 = zeros(4*Ns,4*Ns);
for i = 1:Nv
     A_gen2(4*i-3:4*i,4*i-3:4*i)=A_sub_gen;
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
%disp(A_gen2(4*Ns-12:4*Ns,4*Ns-12:4*Ns));
%disp(A_gen2(1:12,1:12));
%disp(eyesub(1:12,1:12)); 
disp(eyesub(4*Ns-12:4*Ns,4*Ns-12:4*Ns));
save 'eyesub.mat' eyesub;
%disp(size(A_gen2)); 
%disp(size(Xhat));
%disp(size(eyesub)); 

% longitudinal dynamics
%constraints = [constraints, X(:,k+1) == A*X(:,k) + Su*U(:,k)*ds]; % x_k+1 = Ax_k +\delta x * u
%constraints = [constraints, X2(:,k+1) == A_gen2*X2(:,k)];

% longitudinal dynamics in terms of generalized A
%constraints =[constraints, A_gen*Xhat==0 ];
kprime = 4*(1:Ns-1); % 1, 5 9, 13 ... 
Xkplusone = A_gen*Xhat; 
eoc = 2:Nv;
eyesub*Xhat;
eyeX = eyesub*Xhat; 
disp(size(eyeX));
disp(size(Xkplusone));
for i = 1:4*(Ns-1)
   constraints = [constraints, Xhat(i+4,:) == Xkplusone(i,:)];  
   % todo: rewrite Xhat(i+4,:) in terms of eyesub
   eyeX = eyesub*Xhat; 
  %constraints = [constraints, eyeX(i,:) == Xkplusone(i,:)]; % if eyesub is correct, 
   % then this should give the same results as the first constraint
   % equation
end
%kprime = 1:4*(Ns-1); 
equ = A_gen2*Xhat; 
disp(size(A_gen2*Xhat));
constr = [equ(:,1) == 0]; 
%constraints = [constraints,equ]; 
%for i = 1:4*(Ns-1) 
    % constraints = [constraints, A_gen2*Xhat == 0];   
%end

eq = zeros(3*Nv,Ns);
 for i=1:Nv
     % equality constraints 
     eq(3*i -2, 1) = 0; 
     eq(3*i -1,1) = 1/vstart/Sz; 
     eq(3*i,1) = 0;

     % less than constraints
     constraints=[constraints, -X(3*i,:) <= V(i).axmax*(3*vref*X(3*i-1,:)*Sz - 2)./vref^3/Sdz];
     constraints=[constraints, X(3*i-1,:)<=1/V(i).vxmin/Sz];

     % greater than constraints
     constraints=[constraints, X(3*i-1,:)>= 1/V(i).vxmax/Sz];
     constraints=[constraints, -X(3*i,:)>=amin*(3*vref*X(3*i-1,:)*Sz - 2)./vref.^3/Sdz];     
 end

for i = 1:3*Nv
    constraints = [constraints, X(i,1) == eq(i,1)];
end
     
for i = 1:Nv-1
    constraints = [constraints, 
    X(3*co(i)-2,V(co(i)).Nze) <= X(3*co(i+1)-2,V(co(i+1)).Nzs)]; 
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