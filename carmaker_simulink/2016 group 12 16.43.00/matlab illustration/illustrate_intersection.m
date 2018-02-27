clear all;
clc;

I=intersection;
Nv=3;
ss=[1; 0.4; 1; 1]*14;             % initial positions from the critical zone
sf=[1; 0.7; 1; 1]*25;             % total travel distance
entryangle=[0; 0.5; 1; 1.5]*pi; %[rad] angle at which the vehicles enter the critical zone
exitangle=[0; 1; 1.5; 1.5]*pi;  %[rad] angle at which the vehicles exit the critical zone
V(1:Nv)=standardcar;
for j=1:Nv
    V(j).ss=ss(j);
    V(j).sf=sf(j);
    V(j).entryangle=entryangle(j);
    V(j).exitangle=exitangle(j);
end

%% Open a figure
fig=figure; hold on; box off;

%% plot the intersection
% plot the critical zone
c=I.criticalzone/2; d=I.lanewidth;
fill([-d -c -c -d -d  d d c  c  d  d -d -d], ... 
     [-d -d  d  d  c  c d d -d -d -c -c -d], ...
    0.8*[1 1 1],'EdgeColor',0.7*[1 1 1]);

% plot the road segments
XLIM=[0,0]; YLIM=[0,0];
a=-pi;
for j=1:I.segments
    cosa=cos(a); sina=sin(a);
    
    % plot the road segments. An intersection with 2 roads crossings has 4 road
    % segments.
    if j <= Nv
        p=V(j).ss + I.criticalzone/2 + V(j).length/2 + 1;
    else
        p=1.2*I.criticalzone/2;
    end
    
    % Outer line of one lane
    x=cosa*[p I.lanewidth]-sina*I.lanewidth;
    y=sina*[p I.lanewidth]-cosa*I.lanewidth;
    
    XLIM=[min(XLIM(1), x(1)), max(XLIM(2), x(1))];
    YLIM=[min(YLIM(1), y(1)), max(YLIM(2), y(1))];
    
    plot(x, y, 'k','LineWidth',1.5);
    
    % Outer line of the other lane
    plot(cosa*[p I.lanewidth]+sina*I.lanewidth, ...
        sina*[p I.lanewidth]+cosa*I.lanewidth,'k','LineWidth',1.5);

    % plot the middle line between lanes.
    plot(cosa*[p 0],sina*[p 0],'k--','Color',0.5*[1 1 1]);
    
    a=a+pi/2;
end
h=gca;
xlim(XLIM); ylim(YLIM);
p=get(fig,'Position'); p(3)=p(3)*1.5;
fig.Position=p;
set(h,'Units','pixels','Visible','Off');
p=get(h,'Position');
p(4)=330;
p(3)=p(4)*diff(XLIM)/diff(YLIM);    % keep realistic aspect ratio
set(h,'Position',p);
drawnow; % the positioning of arrow heads does not work well without this line

%% plot vehicles
% Vehicles are positioned counter clockwise, starting from the left most
% road segment on the intersection. 

colors=[0 0 1; 0 1 0; 1 0 0; 0.5 0 0.5; 0.5 0.5 0; 0 0.5 0.5; ];
for j=1:Nv
    % initial positions
    cosa=cos(V(j).entryangle); sina=sin(V(j).entryangle); 
    x0=-cosa*(I.criticalzone/2+V(j).ss) + sina*I.lanewidth/2;
    y0=-sina*(I.criticalzone/2+V(j).ss) - cosa*I.lanewidth/2;
    
   % positions at the entry of intersection
    x1=-cosa*I.lanewidth + sina*I.lanewidth/2;
    y1=-sina*I.lanewidth - cosa*I.lanewidth/2;
    
    % positions at the entry of critical zone
    xL=-cosa*I.criticalzone/2 + sina*I.lanewidth/2;
    yL=-sina*I.criticalzone/2 - cosa*I.lanewidth/2;

    % positions at the exit of intersection
    cosb=cos(V(j).exitangle); sinb=sin(V(j).exitangle); 
    x2=cosb*I.lanewidth + sinb*I.lanewidth/2;
    y2=sinb*I.lanewidth - cosb*I.lanewidth/2;

    % positions at the exit of critical zone
    xH=cosb*I.criticalzone/2 + sinb*I.lanewidth/2;
    yH=sinb*I.criticalzone/2 - cosb*I.lanewidth/2;
    
    % final positions    
    x3=cosb*(V(j).sf-V(j).ss-I.criticalzone/2+10) + sinb*I.lanewidth/2;
    y3=sinb*(V(j).sf-V(j).ss-I.criticalzone/2+10) - cosb*I.lanewidth/2;
    
    s1=sqrt((x1-x0)^2+(y1-y0)^2); % length of path segment from start position to the entry of intersection
    s2=sqrt((x3-x2)^2+(y3-y2)^2); % length of path segment from exit of intersection to final position
    
    % length of path within the intersection
    if abs(cos(V(j).exitangle-V(j).entryangle)) < 1e-3
        % find circular path for vehicles that turn: r^2=(x-a)^2+(y-b)^2
        [a, b, r]=quadfunction([x1,0 ,x2],[y1,0,y2]);
        alim=acos(([x1; x2]-a)/r);          % angles at the entry and exit of intersection
        a12=linspace(alim(1),alim(2),15)';  % several grid poins      
        x12=a + r*cos(a12);
        y12=b + r*sin(a12);       
    else
        x12=[]; y12=[];
    end
    plot([x0; xL],[y0; yL],'Color',colors(j,:));
    plot([xL; x12; xH], [yL; y12; yH],'Color',colors(j,:),'LineWidth',3);
    
    veh=plotveh(x0,y0,V(j).entryangle,V(j).width,V(j).length,colors(j,:));
    
    [figx, figy] = dsxy2figxy(gca, [xH; x3], [yH; y3]);  %(now in figure space)
    annotation('arrow',figx,figy,'HeadStyle','plain', ...
        'Color',colors(j,:),'HeadWidth',6,'HeadLength',7); 
    text(x3,y3-0.8,['$p_',num2str(j),'$'],'Interpreter','Latex','FontSize',12);
    text(xL,yL-1.2,['$L_',num2str(j),'$'],'Interpreter','Latex','FontSize',12);
    text(xH,yH-1.2,['$H_',num2str(j),'$'],'Interpreter','Latex','FontSize',12);
end
text(-3.4,5,'Critical set','FontSize',12);

