function [task] = optimize_task(task)
resopt=struct;
ttot=0;
resopt.cost=Inf;

%OPTIMIZE_TASK Summary of this function goes here
%   Detailed explanation goes here
for j=1:size(task.crossorderperm,1)
    task.crossingorder=task.crossorderperm(j,:);
    res=QPsolveY4(task);
    %res=QPsolveY5(task);
    ttot=ttot+res.time(end);
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

res=resopt;
fprintf('Crossing order: %s, average time=%1.2f ms\n',num2str(res.crossingorder),ttot/size(task.crossorderperm,1)*1000);

end

