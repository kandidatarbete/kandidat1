function [task] = gen_task() 
task=struct;                        % we keep all data here
task.ds=1;                          %[m] sampling interval
%max_s = 110;
max_s = 220;
task.s=[0:task.ds:max_s]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=3;                          % number of vehicles
task.I=intersection;

rng(0,'twister');

% loop on al possible permutations of crossing orders. Note that Yalmip is not
% meant to be called iteratively, so you may want to replace it with
% another algorithm that is better suited for this purpose. 
task.loopcrossorder=false; 
crossingorder=[1:task.Nv]'; % prototype of "first come first served"
randomentryangleadd = randi([1 3],1,task.Nv)'*pi/2;
randomturnangle = randi([-2 2],1,task.Nv)'*pi/2;
ss = []; 
j = 76;
entryangle = [];
vref = [];
for i = 1:task.Nv
    entryangle = [entryangle; 0]; % + (random half integer) * pi mod 2 pi 
    vref = [vref; 50]; %[m/s] reference speed for the vehicles (the first task.Nv elements are used)
    ss = [ss; j]; %[m] distance at which the vehicle enters the critical zone
    j = j+10;
end
entryangle = entryangle + randomentryangleadd; %+ randomentryangleadd;
se=ss+task.I.criticalzone;          %[m] distance at which the vehicle exits the critical zone
%exitangle=mod(entryangle - pi/2,2*pi);  % + (random half integer) * pi mod 2 pi 
exitangle = entryangle;

task.V(1:task.Nv)=standardcar;
for j=1:task.Nv
    task.V(j).s=task.s; % (initial) distance for vehicle j
    task.V(j).ss=ss(j); % distance at which vehicle j enters (starts) the critical zone
    task.V(j).se=se(j); % distance at which vehicle j exits the critical zone
    task.V(j).entryangle=entryangle(j); % entry angle for vehicle j
    task.V(j).exitangle=exitangle(j); % exit angle for vehicle j
    task.V(j).vref=vref(j)*ones(task.Ns,1);  % reference speed for vehicle j
end
%se �ver denna delen f�r att kunna best�mma crossing order
if task.loopcrossorder
    task.crossorderperm=perms(1:task.Nv);
else
    task.crossorderperm=crossingorder(1:task.Nv)';
end

end