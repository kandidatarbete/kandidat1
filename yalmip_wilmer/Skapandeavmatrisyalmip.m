clear all, clc
Ns=100;
Nv=3;
h=0.1;
k=1:Ns-1;
%sdpvar verkar vara ett yalmipverktyg, kanske att matriserna är variabler i
%hpipm
t=sdpvar(Ns,Nv);
z=sdpvar(Ns,Nv);
dz=sdpvar(Ns,Nv);
ddz=sdpvar(Ns,Nv);
%A=zeros(Nv,Nv)
Asub=[1 h 0; 0 1 h; 0 0 1];
%Xtest är en matris med värden för att testa A-matrisen
Xtest=zeros(3*Nv,Ns);
Xtest(2,1)=2;
Xtest(5,1)=3;
Xtest(8,1)=4;

for i=1:Nv
    X(1+(i-1)*3,:)=t(:,i);
    X(2+(i-1)*3,:)=z(:,i);
    X((3+(i-1)*3),:)=dz(:,i);
    
end
%A(1:3,1:3)=Asub
A=zeros(3*Nv,3*Nv);
for i=1:Nv
    %A(i+3*(i-1):3*i-2,i+3*(i-1):i+3*(i-1)+3)=Asub;
    %3*i-2
    %3*i
    A(3*i-2:3*i,3*i-2:3*i)=Asub;
end
B=zeros(3*Nv,1)
u=zeros(3,Ns);
u(1,1)=Xtest(2,1);
u(2,1)=Xtest(5,1);
u(3,1)=Xtest(8,1);
for i =1:Nv
    B(3*i,1)=h;
end
for i=k
    Xtest(:,i+1)=A*Xtest(:,i);
end
plot(Xtest(1,k),k);
hold on
plot(Xtest(4,k),k);
plot(Xtest(7,k),k);
%Xtest(:,k+1)=A*Xtest(:,k);
%X(1,1)
Xtest
Xtest(2,2)