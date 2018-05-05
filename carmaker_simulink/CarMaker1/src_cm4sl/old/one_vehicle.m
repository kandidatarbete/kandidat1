% försök att implementera den otransformerade modellen utan kritiskazon-constraints för ett fordon. 
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


