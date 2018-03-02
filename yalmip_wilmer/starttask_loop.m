yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
clf
clear all;
close all; 
clc;
%close all;

task=struct;                        % we keep all data here
task.ds=1;                          %[m] sampling interval
task.s=[0:task.ds:110]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=3;                          % number of vehicles (cheese within 2-4)
task.I=intersection;

% loop on al possible permutations of crossing orders. Note that Yalmip is not
% meant to be called iteratively, so you may want to replace it with
% another algorithm that is better suited for this purpose. 
task.loopcrossorder=false; 
%crossingorder=[1; 2; 3; 4];   % fixed crossing order. Used when task.loopcrossorder=false (the first task.Nv elements are used)
crossingorder=[1:task.Nv]'; % prototype of "first come first served"
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
entryangle(1) = entryangle(1) + pi/2; 
se=ss+task.I.criticalzone;          %[m] distance at which the vehicle exits the critical zone
exitangle=entryangle;  % + (random half integer) * pi mod 2 pi 

task.V(1:task.Nv)=standardcar;
for j=1:task.Nv
    task.V(j).s=task.s; % (initial) distance for vehicle j
    task.V(j).ss=ss(j); % distance at which vehicle j enters (starts) the critical zone
    task.V(j).se=se(j); % distance at which vehicle j exits the critical zone
    task.V(j).entryangle=entryangle(j); % entry angle for vehicle j
    task.V(j).exitangle=exitangle(j); % exit angle for vehicle j
    task.V(j).vref=vref(j)*ones(task.Ns,1);  % reference speed for vehicle j
end

init;
%se �ver denna delen f�r att kunna best�mma crossing order
if task.loopcrossorder
    task.crossorderperm=perms(1:task.Nv);
else
    task.crossorderperm=crossingorder(1:task.Nv)';
end

%% CVX
resopt=struct;
ttot=0;
resopt.cost=Inf;
for j=1:size(task.crossorderperm,1)
    task.crossingorder=task.crossorderperm(j,:);
    %res=QPsolveY2(task);
    %res=QPsolveY(task); 
    res=QPsolveY4(task);
    %res=QPsolveY5(task);
     ttot=ttot+res.time(end);
    ax=diff(res.v)./diff(res.t); ax=[ax;ax(end,:)];
    fprintf('%s: order=%s, cost=%1.4f, vx~[%1.0f,%1.0f]km/h, ax~[%1.1f,%1.1f]m/s2, t=%1.2f ms\n', ...
        res.status, sprintf('%d',task.crossingorder), res.cost, min(res.v(:))*3.6, max(res.v(:))*3.6, ...
        min(ax(:)), max(ax(:)), res.time(end)*1000);
    
    if res.cost < resopt.cost
        resopt=res;
        resopt.crossingorder=task.crossingorder;
    end
end
res=resopt;
fprintf('Crossing order: %s, average time=%1.2f ms\n',num2str(res.crossingorder),ttot/size(task.crossorderperm,1)*1000);

task.res=res;

animate_res_new;
animate_res_new;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se