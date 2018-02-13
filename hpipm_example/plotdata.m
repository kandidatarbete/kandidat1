% Plot optimization data.
plotspeedtrajectory=true;
plotforces=true;

dc=ocpprob.dc; ds=ocpprob.ds; 

%% plot speed trajectory
if plotspeedtrajectory
    alt=dc.altinit + cumsum(sin(dc.slope))*ds;
    
    figure;  hold on; clear ax h; ax(1)=gca;
    h(1)=fill([dc.s;dc.s(end);0]/1000, [alt-min(alt);0;0],0.9*[1 1 1],...
        'EdgeColor',0.7*[1 1 1],'DisplayName','Road altitude');
    xlim([0 dc.s(end)/1000]); ylabel('Road altitude (m)');
    set(ax(1),'XTick',[],'Layer','top','YAxisLocation','right','YColor','k');

    ax(2) = axes('Position',get(ax(1),'Position'),...
           'Xlim',get(ax(1),'XLim'),'Color','none','XColor','k','YColor','k');      
    h(2)=line(dc.s/1000,ocpprob.Vmin*3.6,'LineStyle','-.','LineWidth',1, ...
    'Parent',ax(2),'DisplayName','Speed limits');
    line(dc.s/1000,ocpprob.Vmax*3.6,'LineStyle','-.','LineWidth',1,'Parent',ax(2));
    line(dc.s/1000,res.v(1:end-1,:)*3.6,'LineStyle','-','LineWidth',1,'Parent',ax(2));
    linkaxes(ax,'x');
    xlabel('Traveled length (km)'); 
    ylabel('Speed (km/h)');
    legend(ax(2),h);
end

%% plot forces
if plotforces
    alt=dc.altinit + cumsum(sin(dc.slope))*ocpprob.ds;
    
    figure;  hold on; clear ax h; ax(1)=gca;
    h(1)=fill([dc.s;dc.s(end);0]/1000, [alt-min(alt);0;0],0.9*[1 1 1],...
        'EdgeColor',0.7*[1 1 1],'DisplayName','Road altitude');
    xlim([0 dc.s(end)/1000]); ylabel('Road altitude (m)');
    set(ax(1),'XTick',[],'Layer','top','YAxisLocation','right','YColor','k');

    ax(2) = axes('Position',get(ax(1),'Position'),...
           'Xlim',get(ax(1),'XLim'),'Color','none','XColor','k','YColor','k');   
    linkaxes(ax,'x');
    
    blim=ocpprob.icetrn.parfmax;
    Femax=min(blim(1) + blim(2)*res.v(1:N).^2 + blim(3)./res.v(1:N), blim(4)*res.v(1:N).^2);
        
    h(2)=line(dc.s/1000,Femax/1000,'LineStyle','-.','LineWidth',1, ...
        'Parent',ax(2),'DisplayName','Force limits');
    h(3)=line(dc.s/1000,res.Fe/1000,'LineStyle','-','LineWidth',1, ...
        'Parent',ax(2),'DisplayName','Engine force');
    h(4)=line(dc.s(1:end-1)/1000,res.Fbrk(1:end-1,:)/1000,'LineStyle','--', ...
        'LineWidth',1,'Parent',ax(2),'Color','r','DisplayName','Braking force');
    
    xlabel('Traveled length (km)'); 
    ylabel('Engine force (kN)');
    legend(ax(2),h,'Location','SouthEast');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2017-12.