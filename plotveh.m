function veh=plotveh(xc,yc,a,width,length,color)
if nargin < 1, xc=0; end;
if nargin < 2, yc=0; end;
if nargin < 3, a=0; end;
if nargin < 4, width=2; end;
if nargin < 5, length=4.5; end;
if nargin < 6, color='r'; end;

veh=hgtransform;

x=[-9,-8.5, 8, 8.7, 9, 9, 8.7, 8,-8.5,-9,-9];
y=[-3,-4,  -4,-3,  -2, 2, 3,   4, 4,   3,-3];
fill(x,y,color,'EdgeColor','k','Parent',veh);

x=[3,3.8,4,3.8,3,-8, -8.5,-8.5,-8,   3,-7,-7.5,-7.5,-7,-0.7,-0.2,0,-0.2,-0.7,  3];
y=[-4,-2,0,2,  4,3.5, 2,  -2,  -3.5,-4,-3,  -2,   2,3,3.6,  2,0,  -2,-3.6,-4];
fill(x,y,'k','EdgeColor','k','Parent',veh);

plot([-2 2],[0 0],'w','Parent',veh);
plot([0 0],[-2 2],'w','Parent',veh);

s=width/8;
set(veh,'Matrix', makehgtform('scale',s)*makehgtform('translate',xc/s,yc/s,0)*makehgtform('zrotate',a));
if ~nargout
    grid on;
    xlim([-1 1]*length/2); ylim([-1 1]*width/2);   
    set(gca,'Units','pixels');
    p=get(gca,'Position'); 
    p(4)=p(3)*width/length;    % keep realistic aspect ratio
    set(gca,'Position',p,'XTick',[],'YTick',[],'Box','off','Visible','off');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se