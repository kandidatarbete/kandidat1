task = gen_task3(); 

% for i = 1:task.Nv
% 
%     ss(i) = j-s(i)-task.I.criticalzone/2; %[m] distance at which the vehicle enters the critical zone
%     se(i)=ss(i)+task.I.criticalzone;

%    task.V(j).ss=ss(j); % distance at which vehicle j enters (starts) the critical zone
%    task.V(j).se=se(j); % distance at which vehicle j exits the critical zone
    
% end







% 
% for j=1:task.Nv
%     task.V(j).Nzs=find(task.s <= task.V(j).ss,1,'last');    % number of samples until the vehicle enters the critical zone
%     task.V(j).Nze=find(task.s >= task.V(j).se,1,'first');   % number of samples until the vehicle leaves the critical zone
%     task.V(j).vrmean=mean(task.V(j).vref);                  % mean reference speed
% end
