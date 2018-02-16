clear all, clc
close all; 
Ns=100; % number of states
Nv=3; % number of vehicles
h=0.1; % timestep
k=1:Ns-1;

test_bound = 10;

% testing out sdpvar
t=sdpvar(Ns,Nv);
z=sdpvar(Ns,Nv);
dz=sdpvar(Ns,Nv);
ddz=sdpvar(Ns,Nv);
%A=zeros(Nv,Nv)

Asub=[1 h 0; 0 1 h; 0 0 1]; % submarix used to build global transfermatrix

% give a few vehicles initial speed
%Xtest=zeros(3*Nv,Ns);
Xtest = sdpvar(3*Nv,Ns);
%Xtest(2,1)=2;
%Xtest(5,1)=3;
%Xtest(8,1)=4;

% constructing state vector
% todo: document indexing
% position is at 1,4,7...
% speed is at 2,5,8... 
% acceleration is at 3,6,9...
for i=1:Nv
    X(1+(i-1)*3,:)=t(:,i);
    X(2+(i-1)*3,:)=z(:,i);
    X((3+(i-1)*3),:)=dz(:,i);
    
end
%A(1:3,1:3)=Asub
A=zeros(3*Nv,3*Nv);

% global A matrix is built with Asub as diagonal elements
for i=1:Nv
    A(3*i-2:3*i,3*i-2:3*i)=Asub;
end

% main rule
for i=k
    constr1 = (Xtest(:,i+1)==A*Xtest(:,i));
end

for i = 1:Nv
   constr2 = (Xtest(3*i -1,:) >= 0);  
end


% all values are temporarily bounded by Xtest\in[0,10]
constraints = [Xtest(:,:) > 0, Xtest(:,:) < test_bound, constr1,constr2]; 
constraints = [constraints, -1 <= [t z dz] <= 1];

cost = sum(X(3,:));
%cost = []; 

%constraints = [constraints, Xtest(8,:) <= 1]; % needs to be an sdpvar


% run the optimization 
options     = sdpsettings('verbose',0,'debug', 1); 
sol         = optimize(constraints, sum(cost), options); 




% Analyze error flags
if sol.problem == 0
 % Extract and display value
 solution = value(Xtest); 
 display('succesful optimization run, type value(Xtest) for output values');
else
 display('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end
X = value(Xtest);
%plot the optimized trajectories
figure(2);
%plot(X(1,k),k);
plot(k, X(1,k),'r');
hold on
%plot(X(4,k),k);
plot(k,X(4,k),'g'); 
%plot(X(7,k),k); 
plot(k,X(7,k),'b');
legend('car1','car2', 'car3'); 

