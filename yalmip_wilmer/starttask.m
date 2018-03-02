yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
clf
clear all;
clc;
%close all;

task=struct;                        % we keep all data here
task.ds=1;                          %[m] sampling interval
task.s=[0:task.ds:110]';            %[m] vector of traversed distance
task.Ns=numel(task.s);
task.Nv=4;                          % number of vehicles (cheese within 2-4)
task.I=intersection;

% loop on al possible permutations of crossing orders. Note that Yalmip is not
% meant to be called iteratively, so you may want to replace it with
% another algorithm that is better suited for this purpose. 
task.loopcrossorder=false; 
%crossingorder=[1; 2; 3; 4];   % fixed crossing order. Used when task.loopcrossorder=false (the first task.Nv elements are used)
crossingorder=[1:task.Nv]'; % prototype of "first come first served"

ss=[76; 78; 80; 75; 80; 80];        %[m] distance at which the vehicle enters the critical zone (the first task.Nv elements are used)
se=ss+task.I.criticalzone;          %[m] distance at which the vehicle exits the critical zone
entryangle=[0; 0.5; 1; 1.5;0;0]*pi; %[rad] angle at which the vehicles enter the critical zone
exitangle=[0; 1; 1.5; 1.5; 0;0]*pi; %[rad] angle at which the vehicles exit the critical zone
vref=[47; 48; 50; 49; 50; 50]/3.6;  %[m/s] reference speed for the vehicles (the first task.Nv elements are used)

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
    
    %res=QPsolveY2(task); ttot=ttot+res.time(end);
    %res=QPsolveY(task); ttot=ttot+res.time(end);
    res=QPsolveY4(task); ttot=ttot+res.time(end);
    %res=QPsolveY5(task);
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
% save task.mat task;

%% some plots
% speed
f=figure; 
f.Position(4)=700; f.Position(2)=70;
subplot(3,1,1); 
h=plot(task.s,res.v*3.6);
legend(h,'Location','NorthWest');
xlabel('Position (m)');
ylabel('Speed (km/h)');
colors=get(gca,'DefaultAxesColorOrder');

% acceleration
subplot(3,1,2); 
axmin=-repmat([task.V.axmax],task.Ns,1).*(3*[task.V.vref]./res.v-2).*(res.v./[task.V.vref]).^3;
axmax=-repmat([task.V.axmin],task.Ns,1).*(3*[task.V.vref]./res.v-2).*(res.v./[task.V.vref]).^3;
h=plot(task.s,ax); hold on; grid on;
hacc=plot(task.s,axmax,'k-.','DisplayName','Acceleration limits');
h(end+1)=hacc(1);
plot(task.s,axmin,'k-.');
legend(h,'Location','SouthEast');
xlabel('Position (m)');
ylabel('Acceleration (m/s^2)');

% time
subplot(3,1,3); 
set(gca, 'ColorOrder', [0 0 0; colors], 'NextPlot', 'replacechildren');
co=res.crossingorder(1:task.Nv); co=co(:);
xix=[[task.V(co).Nzs]'; [task.V(flipud(co)).Nze]'; task.V(co(1)).Nzs]; 
yix=[co; flipud(co); co(1)];
x=[ss(co); se(flipud(co)); ss(co(1))]; y=res.t(sub2ind(size(res.t),xix,yix));
h1=fill(x,y,0.8*[1 1 1],'EdgeColor',0.7*[1 1 1],'DisplayName','Critical zone');  hold on; grid on;
h2=plot(task.s,res.t); hold on;
legend([h1;h2],'Location','NorthWest');
xlabel('Position (m)'); 
ylabel('Time (s)');

animate_res;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se