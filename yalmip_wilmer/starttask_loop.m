yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
clf;
clear all;
close all; 
clc;

task=gen_task();
init;
task = optimize_task(task);
disp('animating task'); 

looporder=2;
for (i =1:looporder)
    animate_res_new(task);
    % rebuild task from previous task and reoptimize
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se