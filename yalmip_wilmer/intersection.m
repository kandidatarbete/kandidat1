function I=intersection
%INTERSECTION  Load parameters for a standard intersection
%
% I = INTERSECTION returns a struct I that contains parameters for an
% intersection.

I=struct;
I.segments=4;               % number of road segments intersecting (2 roads crossing create 4 segments)
I.numlanes=2;               % number of lanes
I.lanewidth=5;              %[m] lane width
I.speedlimit=70/3.6;        %[m/s] speed limit
I.criticalzone=25;          %[m] size of critical zone

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-10.
%   nikolce.murgovski@chalmers.se