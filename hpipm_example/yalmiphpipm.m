clear all,clc;
load ocpprob;

task=struct;                        % we keep all data here
task.ds=1;                          %[m] sampling interval
task.s=[0:task.ds:110]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=3;                          % number of vehicles (cheese within 2-4)
task.I=intersection;

task.loopcrossorder=true;           
crossingorder=[1; 2; 3; 4; 5; 6];   % fixed crossing order. Used when task.loopcrossorder=false (the first task.Nv elements are used)

ss=[76; 78; 80; 75; 80; 80];        %[m] distance at which the vehicle enters the critical zone (the first task.Nv elements are used)
se=ss+task.I.criticalzone;          %[m] distance at which the vehicle exits the critical zone


%Dessa anger var vi kommer in, vart vi åker och hur snabbt vi åker i början
entryangle=[0; 0.5; 1; 1.5;0;0]*pi; %[rad] angle at which the vehicles enter the critical zone
exitangle=[0; 1; 1.5; 1.5; 0;0]*pi; %[rad] angle at which the vehicles exit the critical zone
vref=[47; 48; 50; 49; 50; 50]/3.6;  %[m/s] reference speed for the vehicles (the first task.Nv elements are used)
%Nv innebär number of vehicles
task.V(1:task.Nv)=standardcar;
for j=1:task.Nv
    task.V(j).s=task.s;
    task.V(j).ss=ss(j);
    task.V(j).se=se(j);
    task.V(j).entryangle=entryangle(j);
    task.V(j).exitangle=exitangle(j);
    task.V(j).vref=vref(j)*ones(task.Ns,1);    
end

init;
%ändra denna för att bestämma crossingorder
if task.loopcrossorder
    task.crossorderperm=perms(1:task.Nv);
else
    %permantent crossingorder
    task.crossorderperm=crossingorder(1:task.Nv)';
end

%% optimering
resopt=struct;
ttot=0;
resopt.cost=Inf;


for j=1:size(task.crossorderperm,1)
    task.crossingorder=task.crossorderperm(j,:);
    %TODO gör om qpsolver till hpipm
    %resY=QpsolveY(task);
    resH=QPsolveH(task);
    res = resH;
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
