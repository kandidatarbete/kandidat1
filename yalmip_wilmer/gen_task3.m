function task = gen_task3() 
task=struct;

                        % we keep all data here
task.ds=1;                          %[m] sampling interval
%max_s = 110;
max_s = 220;
task.s=[0:task.ds:max_s]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=20;                          % number of vehicles
task.I=intersection;

task.V(1:task.Nv)=standardcar;
%rng(0,'twister');

%%

% loop on al possible permutations of crossing orders. Note that Yalmip is not
% meant to be called iteratively, so you may want to replace it with
% another algorithm that is better suited for this purpose. 
task.loopcrossorder=false; 
crossingorder=[1:task.Nv]'; % prototype of "first come first served"

ss = []; 
j=150; %distance to center of intersection
task.I.criticalzone;
%vref hardcoded to 50 for all vehicles TODO
vref = ones(task.Nv)*50; %[m/s] reference speed for the vehicles (the first task.Nv elements are used)

for j=1:task.Nv
    
    task.V(j).vref=vref(j)*ones(task.Ns,1);  % reference speed for vehicle j
end
%se �ver denna delen f�r att kunna best�mma crossing order
if task.loopcrossorder
    task.crossorderperm=perms(1:task.Nv);
else
    task.crossorderperm=crossingorder(1:task.Nv)';
end




% Initialise data
%% Critical zone

%% Scaling factors
vrmean=task.V(1).vref(1);
task.St=task.Ns*task.ds/vrmean; 
task.Sz=1./vrmean; 
task.Sdz=mean(-[task.V.axmin]./[vrmean].^3);
task.Sddz=50/vrmean^5;
task.Scost=3*vrmean; 

%% Penalties
task.Wv=1; task.Wdv=1; task.Wddv=0.5;




end