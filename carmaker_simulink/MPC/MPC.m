function [res] = MPC([a],[v],[s], [entryangle], [exitangle])
    %FULMPC Summary of this function goes here
    %   Wrapper för optimize task samt interface mot simulink
    % v s och a kolonnvektorer
    yalmip_addpath;                     % add path to Yalmip and solver Ecos. This needs to be done once after opening Matlab.
    clf;
    clear all;
    close all;
    clc;

    task=gen_task2(a,v,s,entryangle,exitangle,false);
    task = optimize_task(task);

    res = 1;
end

