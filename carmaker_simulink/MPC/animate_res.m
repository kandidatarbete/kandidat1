% clear all;
% clc;
saveobj=false; % if true, it will save a video of the animation
res=task.res; I=task.I;
V=task.V;

%% Open a figure
if exist('fig','var') && ishandle(fig)
    figure(fig);
else
    fig=figure('units','normalized','outerposition',[0 0 1 1]); hold on;
end
set(gca,'Unit','normalized','Position',[0.05 0.05 0.94 0.895],'Layer','top', ...
    'XLimMode','manual','YLimMode','manual','ZLimMode','manual');
set(gca,'Units','pixels','Visible','Off');

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
    
    % start position of the road segment 
    if j <= task.Nv
        p=V(j).ss + I.criticalzone/2 + V(j).length/2 + 1;
    else
        p=1.5*I.criticalzone/2;
    end
    
    % Outer line of first lane
    x=cosa*[p I.lanewidth]-sina*I.lanewidth;
    y=sina*[p I.lanewidth]-cosa*I.lanewidth;
    
    XLIM=[min(XLIM(1), x(1)), max(XLIM(2), x(1))];
    YLIM=[min(YLIM(1), y(1)), max(YLIM(2), y(1))];
    
    plot(x, y, 'k','LineWidth',1.5);
    
    % Outer line of second lane
    plot(cosa*[p I.lanewidth]+sina*I.lanewidth, ...
        sina*[p I.lanewidth]+cosa*I.lanewidth,'k','LineWidth',1.5);

    % plot the line separating the lanes.
    plot(cosa*[p 0],sina*[p 0],'k--');
    
    a=a+pi/2;
end
h=gca;
xlim(XLIM); ylim(YLIM);
p=get(h,'Position');
p(1)=150;                        % move it near the center of screen
% p(4)=p(3)*diff(YLIM)/diff(XLIM);    % keep realistic aspect ratio
p(3)=p(4)*diff(XLIM)/diff(YLIM);    % keep realistic aspect ratio
set(h,'Position',p);
drawnow; % the positioning of arrow heads does not work well without this line

%% plot vehicles
% Vehicles are positioned counter clockwise, starting from the left most
% road segment on the intersection. 

% colors=get(gca,'DefaultAxesColorOrder');
colors=[0 0 1; 0 1 0; 1 0 0; 0.5 0 0.5; 0.5 0.5 0; 0 0.5 0.5; 0 0 1; 0 0 1; 0 0 1; 0 0 1; 0 0 1; 0 0 1; 0 0 1];
for j=1:task.Nv
    % initial positions
    cosa=cos(V(j).entryangle); sina=sin(V(j).entryangle); 
    x0=-cosa*(I.criticalzone/2+V(j).ss) + sina*I.lanewidth/2;
    y0=-sina*(I.criticalzone/2+V(j).ss) - cosa*I.lanewidth/2;
    
    % positions at the entry of intersection
    x1=-cosa*I.lanewidth + sina*I.lanewidth/2;
    y1=-sina*I.lanewidth - cosa*I.lanewidth/2;

    % positions at the exit of intersection
    cosb=cos(V(j).exitangle); sinb=sin(V(j).exitangle); 
    x2=cosb*I.lanewidth + sinb*I.lanewidth/2;
    y2=sinb*I.lanewidth - cosb*I.lanewidth/2;
    
    % final positions    
    x3=cosb*(task.s(end)-V(j).ss-I.criticalzone/2+10) + sinb*I.lanewidth/2;
    y3=sinb*(task.s(end)-V(j).ss-I.criticalzone/2+10) - cosb*I.lanewidth/2;
    
    s1=sqrt((x1-x0)^2+(y1-y0)^2); % length of path segment from start position to the entry of intersection
    s2=sqrt((x3-x2)^2+(y3-y2)^2); % length of path segment from exit of intersection to final position
    
    % length of path within the intersection
    if abs(cos(V(j).exitangle-V(j).entryangle)) < 1e-3
        % find circular path for vehicles that turn: r^2=(x-a)^2+(y-b)^2
        [a, b, r]=quadfunction([x1,0 ,x2],[y1,0,y2]);
        alim=acos(([x1; x2]-a)/r);          % angles at the entry and exit of intersection
        a12=linspace(alim(1),alim(2),10)';  % several grid poins
        
        arc=linspace(0,r*abs(diff(alim)),10)'; % length of the arc within the intersection
        x12=a + r*cos(a12);
        y12=b + r*sin(a12);       
        
        V(j).path.s=[0; s1-0.01; s1+arc; s1+arc(end)+0.01; s1+arc(end)+s2];
        V(j).path.x=[x0; x12(1); x12; x12(end); x3];
        V(j).path.y=[y0; y12(1); y12; y12(end); y3];
        V(j).path.angle=[V(j).entryangle; V(j).entryangle; a12 + pi/2; V(j).exitangle; V(j).exitangle];
    else
        % paths are straight lines
        V(j).path.s=[0;s1+s2+2*I.lanewidth];
        V(j).path.x=[x0; x3];
        V(j).path.y=[y0; y3];
        V(j).path.angle=[V(j).entryangle; V(j).exitangle];
    end
    plot(V(j).path.x(1:end-1),V(j).path.y(1:end-1),'Color',colors(j,:));
    [figx, figy] = dsxy2figxy(gca, V(j).path.x(end-1:end), V(j).path.y(end-1:end));  %(now in figure space)
    %annotation('arrow',figx,figy,'HeadStyle','plain', ...
     %   'Color',colors(j,:),'HeadWidth',6,'HeadLength',7); 
end

% plot the vehicles at their initial position
vehg=gobjects(task.Nv);
txtspeed=gobjects(task.Nv);
for j=1:task.Nv
    vehg(j)=hgtransform('Parent',h);
    veh=plotveh(V(j).path.x(1),V(j).path.y(1),V(j).path.angle(1),V(j).width,V(j).length,colors(j,:)); set(veh,'Parent',vehg(j));
    txtspeed(j)=text(V(j).path.x(1) + 3,V(j).path.y(1)+ 3,sprintf('%1.0f km/h',res.v(1,j)*3.6),'Parent',vehg(j),'Color',colors(j,:));    
end

%% Animate
% return;
if saveobj
    disp("Saving video of animation. This could slow down the animation considerably");
    %writerObj=VideoWriter('task.mp4','MPEG-4');
    writerObj=VideoWriter('./videos/task.avi'); 
    writerObj.FrameRate=20;
    writerObj.Quality=100;
    open(writerObj);
    
    Nt=300; % number of time samples   
else
    Nt=600;
end
x=NaN(Nt,task.Nv); y=NaN(Nt,task.Nv); angle=NaN(Nt,task.Nv); v=NaN(Nt,task.Nv);

% create a time vector with high resolution
t=linspace(0,max(res.t(:)),Nt)';

% linear interpolation in Matlab 2017b has a bug. The problem seems to b
% avoided if initial time is identically zero (the optimal result may
% deviate slightly due to numerical errors).
res.t(1,:)=0;

t=max(0,t);
for j=1:task.Nv
    s=interp1(res.t(:,j),task.s,t);
    x(:,j)=interp1(V(j).path.s,V(j).path.x,s);
    y(:,j)=interp1(V(j).path.s,V(j).path.y,s);
    angle(:,j)=interp1(V(j).path.s,V(j).path.angle,s);
    v(:,j)=interp1(task.s,res.v(:,j),s);
end

% return;
for k=2:Nt
    for j=1:task.Nv
        if t(k) <= res.t(end,j)
            % transformations are performed from right to left. It is
            % better to first translate the system to origin, then perform
            % rotations, and translate back to desired position.
            vehg(j).Matrix=makehgtform('translate',x(k,j),y(k,j),0,'zrotate',angle(k,j)-angle(1,j),'translate',-x(1,j),-y(1,j),0);
            % equivalent transformation is vehg.Matrix=makehgtform('translate',x(k,j),y(k,j),0,'zrotate',angle(k,j),'zrotate',-angle(1,j),'translate',-x(1,j),-y(1,j),0);
            txtspeed(j).String=sprintf('%1.0f km/h,', v(k,j)*3.6);
        else
            vehg(j).Visible='off';
        end
    end
    
    if saveobj
%         frame = getframe(h,[-50 -50 h.Position(3)+50 h.Position(4)+50]);
        frame = getframe(h);
        writeVideo(writerObj,frame);
    else
        drawnow update;
%         drawnow;
        pause(1e-2)
    end
end
if saveobj
    close(writerObj);
end

function [a, b, r]=quadfunction(x,y)
    % Computes circle parameters according to r^2=(x-a)^2 + (y-b)^2, given
    % points x, y.

    x21 = x(2)-x(1); y21 = y(2)-y(1);
    x31 = x(3)-x(1); y31 = y(3)-y(1);
    h21 = x21^2+y21^2; h31 = x31^2+y31^2;
    d = 2*(x21*y31-x31*y21);
    a = x(1)+(h21*y31-h31*y21)/d;
    b = y(1)-(h21*x31-h31*x21)/d;
    r = sqrt(h21*h31*((x(3)-x(2))^2+(y(3)-y(2))^2))/abs(d);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se