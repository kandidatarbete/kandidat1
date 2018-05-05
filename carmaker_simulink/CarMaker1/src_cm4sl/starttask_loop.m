<<<<<<< HEAD
=======

>>>>>>> 203dc61d720575e64c767c0d7c10c048d2193576
yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
clf;
clear all;
close all; 
clc;
<<<<<<< HEAD

looporder=9;
for i =1:looporder
    task=gen_task(false);
    task = optimize_task(task);
    animate_res_new(task);
end

=======
time=cputime;

looporder=1;
for i =1:looporder
    task=gen_task(false);
    task = optimize_task(task);
    disp(cputime-time)  
    animate_res_new(task);
end


>>>>>>> 203dc61d720575e64c767c0d7c10c048d2193576
