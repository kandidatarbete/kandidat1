%%
task = struct;
task.ds=1;                          %[m] sampling interval
task.s=[0:task.ds:110]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=3;                          % number of vehicles (cheese within 2-4)
task.I=intersection;


% loop on al possible permutations of crossing orders. Note that Yalmip is not
% meant to be called iteratively, so you may want to replace it with
% another algorithm that is better suited for this purpose. 
task.loopcrossorder=true;           
crossingorder=[1; 2; 3; 4; 5; 6];   % fixed crossing order. Used when task.loopcrossorder=false (the first task.Nv elements are used)

ss=[76; 78; 80; 75; 80; 80];        %[m] distance at which the vehicle enters the critical zone (the first task.Nv elements are used)
se=ss+task.I.criticalzone;          %[m] distance at which the vehicle exits the critical zone
entryangle=[0; 0.5; 1; 1.5;0;0]*pi; %[rad] angle at which the vehicles enter the critical zone
exitangle=[0; 1; 1.5; 1.5; 0;0]*pi; %[rad] angle at which the vehicles exit the critical zone
vref=[47; 48; 50; 49; 50; 50]/3.6;  %[m/s] reference speed for the vehicles (the first task.Nv elements are used)




task.V(1:task.Nv)=standardcar;
for j=1:task.Nv
    task.V(j).s=task.s;
    task.V(j).ss=ss(j);
    task.V(j).se=se(j);
    task.V(j).entryangle=entryangle(j);
    task.V(j).exitangle=exitangle(j);
    task.V(j).vref=vref(j)*ones(task.Ns,1);    
end



% Critical zone
for j=1:task.Nv
    task.V(j).Nzs=find(task.s <= task.V(j).ss,1,'last');    % number of samples until the vehicle enters the critical zone
    task.V(j).Nze=find(task.s >= task.V(j).se,1,'first');   % number of samples until the vehicle leaves the critical zone
    task.V(j).vrmean=mean(task.V(j).vref);                  % mean reference speed
end

% Scaling factors
vrmean=mean([task.V.vrmean]);
task.St=task.Ns*task.ds/vrmean; 
task.Sz=1./vrmean; 
task.Sdz=mean(-[task.V.axmin]./[task.V.vrmean].^3);
task.Sddz=50/vrmean^5;
task.Scost=3*vrmean; 

% Penalties
task.Wv=1; task.Wdv=1; task.Wddv=0.5;


%%
clear all, clc
close all; 

Ns = 100; % number of samples
Nv = 2; 
k=1:Ns-1;
% yalmip-specifikt, referera hädanefter uteslutande till X(i), index i 
% specificerar fordon och variabel
for i = 1:Nv 
t(i,:) = sdpvar(1,Ns); 
p(i,:) = sdpvar(1,Ns);
v(i,:) = sdpvar(1,Ns); 
a(i,:) = sdpvar(1,Ns); 
end
for i=1:Nv
    %U(3*i-2)=0;
    %U(3*i-1)=0;
    U(3*i,:)=sdpvar(1,Ns);
end
 % styrvariabeln
for i=1:Nv
    
    X(3*i-2,:)=p(i,:);
    X(3*i-1,:)=v(i,:);
    X(3*i,:)=a(i,:);
    %X = [p(i,:); v(i,:); a(i,:)]; % planeringsrapporten sidan 1
end
%X = [p(1,:); v(1,:); a(1,:)]; % planeringsrapporten sidan 1
%A = [0 1 0; 0 0 1; 0 0 0];
h = 0.1;
Asub = [1 h 0; 0 1 h; 0 0 0]; 

% konstruera generaliserade A: 
for i=1:Nv
    A(3*i-2:3*i,3*i-2:3*i)=Asub;
end

% definera X(t=0)
X(2) = 0.1; % initial speed
X(1) = 0; % initial position
X(3) = 0; % initial acceleration

% huvudregel
% X_{k+1} = A*X_k \forall k, en constraint per sample
constraints  = [];
for k=1:Ns-1
    constraints = [constraints, X(:,k+1) == A*X(:,k) + U(:,k)*h];  % Ax + h * u 
   
    
end

% ett fordon kan inte gasa/bromsa hur hårt som helst
constraints = [constraints, (-5 <= a <= 5)]; 

constraints = [constraints, (-5 <= U(:,k) <= 5)]; 

cost = [sum(X(1,:))]; % se till att dom kommer så långt som möjligt

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

value(X)

% plot 


for i=1:Nv
    plot(1:Ns,value(X(3*i-2,:)));
    hold on
end
%plot(1:Ns,value(X(1,:)));
hold on
%plot(1:Ns,value(X(4,:)));
xlabel('sample no');
ylabel('position'); 


