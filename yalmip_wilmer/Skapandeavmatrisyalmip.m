clear all, clc
Ns=100; % number of states
Nv=3; % number of vehicles
h=0.1; % timestep
k=1:Ns-1;

% testing out sdpvar
t=sdpvar(Ns,Nv);
z=sdpvar(Ns,Nv);
dz=sdpvar(Ns,Nv);
ddz=sdpvar(Ns,Nv);
%A=zeros(Nv,Nv)

Asub=[1 h 0; 0 1 h; 0 0 1]; % submarix used to build global transfermatrix

% give a few vehicles initial speed
Xtest=zeros(3*Nv,Ns);
Xtest(2,1)=2;
Xtest(5,1)=3;
Xtest(8,1)=4;

% constructing state vector
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
for i=k
    Xtest(:,i+1)=A*Xtest(:,i);
end

% plot trajectories
plot(Xtest(1,k),k);
hold on
plot(Xtest(4,k),k);
plot(Xtest(7,k),k);
%Xtest(:,k+1)=A*Xtest(:,k);
%X(1,1)
Xtest
Xtest(2,2)