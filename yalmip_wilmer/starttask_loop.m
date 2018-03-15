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

looporder=9;
for (i =1:looporder)
    %animate_res_new(task);
    animate_res_new;
%     rebuild task from previous task and reoptimize
     past=past_criticalzone(task,task.Ns);%returnar en array som s�ger vilka som �r utanf�r
%      for j=1:task.Nv
%          if(past(j))
%              task=remove_vehicle(j,task);
%          end
%      end
    %task = remove_vehicle(1,task); 
    task = optimize_task(task);
end

