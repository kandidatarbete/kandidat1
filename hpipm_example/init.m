% Initialise data

%% Critical zone
for j=1:task.Nv
    task.V(j).Nzs=find(task.s <= task.V(j).ss,1,'last');    % number of samples until the vehicle enters the critical zone
    task.V(j).Nze=find(task.s >= task.V(j).se,1,'first');   % number of samples until the vehicle leaves the critical zone
    task.V(j).vrmean=mean(task.V(j).vref);                  % mean reference speed
end

%% Scaling factors
vrmean=mean([task.V.vrmean]);
task.St=task.Ns*task.ds/vrmean; 
task.Sz=1./vrmean; 
task.Sdz=mean(-[task.V.axmin]./[task.V.vrmean].^3);
task.Sddz=50/vrmean^5;
task.Scost=3*vrmean; 

%% Penalties
task.Wv=1; task.Wdv=1; task.Wddv=0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se