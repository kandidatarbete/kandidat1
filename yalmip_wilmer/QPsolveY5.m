function res=QPsolveY5(task)
V=task.V; Ns=task.Ns; Nv=task.Nv; ds=task.ds; 
co=task.crossingorder(1:Nv);

% scaling factors 
St=task.St; Sz=task.Sz; Sdz=task.Sdz; Sddz=task.Sddz; Scost=task.Scost;

%penalties
Wv=task.Wv; Wdv=task.Wdv; Wddv=task.Wddv;
%%
yalmip('clear')
%TODO vref ska inte vara hårdkodad
vref=50;
vstart=12;
amin=-5;
%bromstid
deltat=20;
k=1:Ns-1;
% for i=1:Nv
% t(i,:) = sdpvar(1,Ns,'full'); 
% z(i,:) = sdpvar(1,Ns,'full');
% dz(i,:) = sdpvar(1,Ns,'full'); 
% ddz(i,:) = sdpvar(1,Ns,'full'); 
%     
% end
X=sdpvar(3*Nv,Ns,'full');



% control signal for acceleration
for i=1:Nv
    U(3*i,:)=sdpvar(1,Ns);
end


% main state variable
% 
% for i=1:Nv
%     X(3*i-2,:)=t(i,:);
%     X(3*i-1,:)=z(i,:);
%     X(3*i,:)=dz(i,:);
% end

 % step size
Asub = [1 ds 0; 0 1 ds; 0 0 1]; % A matrix for 1 vehicle 

% konstruera generaliserade A: 
for i=1:Nv
    A(3*i-2:3*i,3*i-2:3*i)=Asub;
end

% scaling factors
Su=zeros(3*Nv);
SuSub=eye(3);
SuSub(1,1)=Sz/St;
SuSub(2,2)=Sdz/Sz;
SuSub(3,3)=Sddz/Sdz;
for i=1:Nv
    Su(3*i-2:3*i,3*i-2:3*i)=SuSub;
end

constraints  = []; 
constraints = [constraints, X(:,k+1) == A*X(:,k) + Su*U(:,k)*ds]; % x_k+1 = Ax_k +\delta x
for i=1:Nv
     %constraints=[constraints, X(3*i-1,:)>=0]; % lethargy > 0 
      %constraints=[constraints, 
      %X(3*i-2,2:Ns)==X(3*i-2,1:Ns-1)+ h*X(3*i-1,1:Ns-1)*Sz/St];
     constraints=[constraints, X(3*i-2,1)==0];
     %TODO borde vara vstart, inte vref
     constraints=[constraints, X(3*i-1,1)==1/vstart/Sz];
     constraints=[constraints, X(3*i-1,:)>= 1/V(i).vxmax/Sz];
     %TODO nedanstående verkar orsaka lite problem
     constraints=[constraints, X(3*i-1,:)<=1/V(i).vxmin/Sz];
     constraints=[constraints, X(3*i,1)==0];
     constraints=[constraints, -X(3*i,:)>=amin*(3*vref*X(3*i-1,:)*Sz - 2)./vref.^3/Sdz];
     constraints=[constraints, -X(3*i,:) <= V(i).axmax*(3*vref*X(3*i-1,:)*Sz - 2)./vref^3/Sdz];
     
end
% critical zone constraints
% for j=1:Nv-1
%     constraints =  [constraints, ...
%         t(V(co(j)).Nze, co(j)) <= t(V(co(j+1)).Nzs,co(j+1)) ];
% end
%critical zone constraint 
for i = 1:Nv-1
    %T(V(co(j)) = X(3*i-2);
    constraints = [constraints, 
    X(3*co(i)-2,V(co(i)).Nze) <= X(3*co(i+1)-2,V(co(i+1)).Nzs)]; 
end
%constraints för att kolla så att fordon inte kör in i varandra bakifrån
%fungerar inte nu då vi inte kan starta bilar på samma väg utan att de är i
%varandra

for i=1:Nv-1
    for j=1:Nv
        if V(i).entryangle==V(j).entryangle && i~=j
            [row1,column]=find(crossingorder==i);
            [row2,column]=find(crossingorder==j);
            if row1<row2
                constraints=[constraints, X(3*i-2,:)>=X(3*j-2,:)+deltat];
            else
                constraints=[constraints, X(3*j-2,:)>=X(3*i-2,:)+deltat];
            end
        
        end
    end
    
end


cost=[];
cost1=[];
cost2=[];
cost3=[];
for i=1:Nv
cost1 = [cost1, Wv*vref^3.*sum((X(3*i-1,:)-1/vref).^2)]; % equation 4a
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
    display('naj');
end


for i=1:Nv
    subplot(1,Nv,i)
    plot(value(X(3*i-2,:)),1./value(X(3*i-1,:)));
    
    xlabel('time');
    ylabel('velocity'); 

    
    
    %plot(1:Ns,value(X(3*i-1,:)));
    hold on
end

end