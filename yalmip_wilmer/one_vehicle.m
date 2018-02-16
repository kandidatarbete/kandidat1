% försök att implementera den otransformerade modellen utan kritiska
% zon-constraints för ett fordon. 
clear all, clc
close all; 

Ns = 100; % number of samples

t = sdpvar(1,Ns); 
p = sdpvar(1,Ns);
v = sdpvar(1,Ns); 
a = sdpvar(1,Ns); 

X = [p; v; a]; % planeringsrapporten sidan 1
%A = [0 1 0; 0 0 1; 0 0 0];
h = 0.1
A = [1 h 0; 0 1 h; 0 0 0]; 

% definera X(t=0)
X(2) = 5; % initial speed
X(1) = 0; % initial position
X(3) = 0; % initial acceleration

% huvudregel
% X_{k+1} = A*X_k \forall k, en constraint per sample och variabel
constraints  = [];
for k = 1:(Ns-1)
    constraints = [constraints, X(:,k+1) == A*X(:,k)];  % + h * u 
   
    
end

cost = []; 

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