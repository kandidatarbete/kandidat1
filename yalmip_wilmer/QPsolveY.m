function res=QPsolveY(task)
% res=QPsolveY(task) returns the optimal result, res. The
% optimization task is provided in the structure task.

V=task.V; Ns=task.Ns; Nv=task.Nv; ds=task.ds; 
co=task.crossingorder(1:Nv);

% scaling factors 
St=task.St; Sz=task.Sz; Sdz=task.Sdz; Sddz=task.Sddz; Scost=task.Scost;

%penalties
Wv=task.Wv; Wdv=task.Wdv; Wddv=task.Wddv;

%% YALMIP
yalmip('clear')
% Define variables
z = sdpvar(Ns,Nv);      % lethargy (inverse speed)
t = sdpvar(Ns+1,Nv);    % time
dz = sdpvar(Ns-1,Nv);   % 1st derivative of lethargy
ddz = sdpvar(Ns-2,Nv);  % 2nd derivative of lethargy

constraints = []; 
for j=1:Nv
    cost(j)=(sum((z(:,j)*Sz - 1./V(j).vref).^2)*Wv*V(j).vrmean^3 ...
        + sum(dz(:,j).^2)*Sdz^2*Wdv*V(j).vrmean^5 ...
        + sum(ddz(:,j).^2)*Sddz^2*Wddv*V(j).vrmean^7)*ds/Scost;
    
    constraints =  [constraints, ...
        t(2:Ns+1,j) == t(1:Ns,j) + ds*z(1:Ns,j)*Sz/St, ...
        z(2:Ns,j) ==  z(1:Ns-1,j) + ds*dz(1:Ns-1,j)*Sdz/Sz, ...
        dz(2:Ns-1,j) == dz(1:Ns-2,j) + ds*ddz(1:Ns-2,j)*Sddz/Sdz, ...
        z(:,j) >= 1./V(j).vxmax/Sz, ...
        z(:,j) <= 1./V(j).vxmin/Sz, ...
        z(1,j) == 1./V(j).vref(1)/Sz, ...
        dz(1,j) == 0, ...
        -dz(:,j) >= V(j).axmin*(3*V(j).vref(1:Ns-1).*z(1:Ns-1,j)*Sz - 2)./V(j).vref(1:Ns-1).^3/Sdz, ...
        -dz(:,j) <= V(j).axmax*(3*V(j).vref(1:Ns-1).*z(1:Ns-1,j)*Sz - 2)./V(j).vref(1:Ns-1).^3/Sdz, ...
        t(1,j) == 0, ...
        ];
end  

% critical zone constraints
for j=1:Nv-1
    constraints =  [constraints, ...
        t(V(co(j)).Nze, co(j)) <= t(V(co(j+1)).Nzs,co(j+1)) ];
end
     
options     = sdpsettings('verbose',0,'solver','ecos'); 
sol         = optimize(constraints, sum(cost), options); 
  
%% post threat data
if sol.problem == 0   
    res.status='Solved';
    res.time=sol.solvertime;
    res.cost=sum(value(cost));
    res.v=1./value(z)/Sz; 
    res.t=value(t(1:Ns,:))*St;
else
    res.status=sol.info;
    display(sol.info);
end

% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
