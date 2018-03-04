yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
clf;
clear all;
close all; 
clc;

disp('generating task');
task=gen_task();
init;
disp('optimizing task');
task = optimize_task(task);
disp('animating task'); 

looporder=1;
for (i =1:looporder)
    animate_res_new(task);
    % rebuild task from previous task and reoptimize
    past=past_criticalzone(task,current_index);%returnar en array som säger vilka som är utanför
    for(i=1:Nv)
        if(past(i))
            task=remove_vehicle(i,task);
        end
    end
    %task = remove_vehicle(1,task); 
    task = optimize_task(task);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se

% need this data from QPsolve in oder to determine removal/addition of
% vehicles
% 
% for i=1:Nv
% 
% position(i,:)=h.*k;    
% time(i,:)=value(X(3*i-2,:));
% %TODO skalfaktor?
% velocity(i,:)=value(X(3*i-1,:));
% acceleration(i,:)=diff(velocity(i,:))./diff(time(i,:));
% end
% %definiera acceleration
% for i=1:Nv
%    ax=-1./(diff(value(X(3*i-1,:))).*diff(value(X(3*i-2,:))));
% end
% ax(1)=0;
% % plot 


