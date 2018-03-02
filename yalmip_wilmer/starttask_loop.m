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
    task = optimize_task(task);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se