clear all
task = struct;
task.ds=1;                          %[m] sampling interval
task.s=[0:task.ds:110]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=3;                          % number of vehicles (cheese within 2-4)
task.I=intersection;

% loop on all possible permutations of crossing orders. Note that Yalmip is not
% meant to be called iteratively, so you may want to replace it with
% another algorithm that is better suited for this purpose. 
task.loopcrossorder=true;           
crossingorder=[1; 2; 3; 4; 5; 6];   % fixed crossing order. Used when task.loopcrossorder=false (the first task.Nv elements are used)

ss=[76; 78; 80; 75; 80; 80];        %[m] distance at which the vehicle enters the critical zone (the first task.Nv elements are used)
se=ss+task.I.criticalzone;          %[m] distance at which the vehicle exits the critical zone
entryangle=[0; 0.5; 1; 1.5;0;0]*pi; %[rad] angle at which the vehicles enter the critical zone
exitangle=[0; 1; 1.5; 1.5; 0;0]*pi; %[rad] angle at which the vehicles exit the critical zone
%vref=[47; 48; 50; 49; 50; 50]/3.6;  %[m/s] reference speed for the vehicles (the first task.Nv elements are used)
vref=50;

task.V(1:task.Nv)=standardcar;
for j=1:task.Nv
    task.V(j).s=task.s;
    task.V(j).ss=ss(j);
    task.V(j).se=se(j);
    task.V(j).entryangle=entryangle(j);
    task.V(j).exitangle=exitangle(j);
   % task.V(j).vref=vref(j)*ones(task.Ns,1);    
end

% Critical zone
for j=1:task.Nv
    task.V(j).Nzs=find(task.s <= task.V(j).ss,1,'last');    % number of samples until the vehicle enters the critical zone
    task.V(j).Nze=find(task.s >= task.V(j).se,1,'first');   % number of samples until the vehicle leaves the critical zone
   % task.V(j).vrmean=mean(task.V(j).vref);                  % mean reference speed
end

% Scaling factors
%vrmean=mean([task.V.vrmean]);
vrmean=vref;
task.St=task.Ns*task.ds/vrmean; 
task.Sz=1./vrmean; 
task.Sdz=mean(-[task.V.axmin]./[vrmean].^3);
task.Sddz=50/vrmean^5;
task.Scost=3*vrmean; 

% Penalties
task.Wv=1; task.Wdv=1; task.Wddv=0.5;

clc,clf
close all; 
St=task.St;
Sz=task.Sz;
Sdz=task.Sdz;
Sddz=task.Sddz;
vmax=10;
vmin=5;
vstart=6;
amin=-5;
amax=5;
Ns = 100; % number of samples
Nv = 2; 
k=1:Ns-1;
% yalmip-specifikt, referera hädanefter uteslutande till X(i), index i 
% specificerar fordon och variabel
for i=1:Nv
t(i,:) = sdpvar(1,Ns,'full'); 
z(i,:) = sdpvar(1,Ns,'full');
dz(i,:) = sdpvar(1,Ns,'full'); 
ddz(i,:) = sdpvar(1,Ns,'full'); 
    
end

% control signal for acceleration
for i=1:Nv
    U(3*i,:)=sdpvar(1,Ns);
end

% main control variable
for i=1:Nv
    X(3*i-2,:)=t(i,:);
    X(3*i-1,:)=z(i,:);
    X(3*i,:)=dz(i,:);
end

h = 0.4; % step size
Asub = [1 h 0; 0 1 h; 0 0 1]; % A matrix for 1 vehicle 

% konstruera generaliserade A: 
for i=1:Nv
    A(3*i-2:3*i,3*i-2:3*i)=Asub;
end

constraints  = []; 
    
% scaling factors
Su=zeros(3*Nv);
SuSub=eye(3);
SuSub(1,1)=Sz/St;
SuSub(2,2)=Sdz/Sz;
SuSub(3,3)=Sddz/Sdz;
for i=1:Nv
    Su(3*i-2:3*i,3*i-2:3*i)=SuSub;
end
 
constraints = [constraints, X(:,k+1) == A*X(:,k) + Su*U(:,k)*h]; % x_k+1 = Ax_k +\delta x
for i=1:Nv
     %constraints=[constraints, X(3*i-1,:)>=0]; % lethargy > 0 
      %constraints=[constraints, 
      %X(3*i-2,2:Ns)==X(3*i-2,1:Ns-1)+ h*X(3*i-1,1:Ns-1)*Sz/St];
     constraints=[constraints, X(3*i-2,1)==0];
     constraints=[constraints, X(3*i-1,1)==1/vstart/Sz];
     constraints=[constraints, X(3*i-1,:)>= 1/vmax/Sz];
     %TODO nedanst�ende verkar orsaka lite problem
     constraints=[constraints, X(3*i-1,:)<=1/vmin/Sz];
     constraints=[constraints, X(3*i,1)==0];
     constraints=[constraints, -X(3*i,:)>=amin*(3*vref*X(3*i-1,:)*Sz - 2)./vref.^3/Sdz];
     
     
  end

%cost = [1/sum(X(2,:))+1/sum(X(5,:))]; % se till att dom kommer så långt som möjligt
summaZ=0;
summaDdz=0;
for i=1:Nv
    summaZ=summaZ+Sz*sum(X(3*i-1,:));
    summaDdz=summaDdz +Sddz*sum(U(3*i,:)); 
end
%cost=[1/summaZ+ 1/summaDdz];
%cost = [1/summaDdz];
% task.Wv=1; task.Wdv=1; task.Wddv=0.5;
Wv = task.Wv;
Wdv = task.Wdv; 
Wddv = task.Wddv;
cost=[];
cost1=[];
cost2=[];
cost3=[];
for i=1:Nv
cost1 = [cost1, Wv*vref^3.*sum((X(3*i-1,:)-1/vref).^2)]; % equation 4a
cost2 = [cost2, Wdv*vref^5.*sum((X(3*i,:).^2))]; % equation 4b
cost3 = [cost3, Wddv*vref^7.*sum((U(3*i,:).^2))]; % equation 4c
end
cost=[cost,cost1,cost2,cost3];
options     = sdpsettings('verbose',0,'debug', 1); 
sol         = optimize(constraints, sum(cost), options); 

% Analyze error flags
if sol.problem == 0
 % Extract and display value
 solution = value(X); 
 
 display('succesful optimization run, type value(X) for output values');
else
 display('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end

value(X);

% plot 


for i=1:Nv
    figure
    plot(value(X(3*i-2,:)),1./value(X(3*i-1,:)));
    
    %plot(1:Ns,value(X(3*i-1,:)));
    hold on
end
%plot(1:Ns,value(X(1,:)));
hold on
%plot(1:Ns,value(X(4,:)));

xlabel('sample no');
ylabel('lethargy'); 

%value(X(1,:))
