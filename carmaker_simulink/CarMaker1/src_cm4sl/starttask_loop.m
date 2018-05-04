
yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
clf;
clear all;
close all; 
clc;
time=cputime;

looporder=1;
for i =1:looporder
    task=gen_task(false);
    task = optimize_task(task);
    disp(cputime-time)  
    animate_res_new(task);
end


