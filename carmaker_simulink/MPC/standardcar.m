function v=standardcar
%STANDARDCAR  Load parameters for a standard car
%
% V = STANDARDCAR returns a struct V that contains parameters for a car.

v=struct;
v.width=2;                  %[m] vehicle width
v.length=4.5 ;              %[m] vehicle length
v.latsafetymargin=0.5;      %[m] lateral safety margin to surrounding object
v.longsafetymargin=5;       %[m] longitudinal safety margin to surrounding object
v.timelagfront=1;           %[s] minimum time lag to the vehicle in front
v.timelagrear=0.5;          %[s] minimum time lag to the vehicle behind.
v.timelagrearlane=5;        %[s] minimum time lag to the vehicle behind, after which the vehicle has to be in its own lane.
v.timelagfrontlane=5.5;     %[s] minimum time lag to the vehicle in front, before which the vehicle has to be in its own lane.
v.vxmin=30/3.6;             %[m/s] minimum longitudinal speed
v.vxmax=90/3.6;             %[m/s] maximum longitudinal speed
v.vymin=-4;                 %[m/s] minimum lateral speed
v.vymax=4;                  %[m/s] maximum lateral speed
v.axmin=-3;                 %[m/s2] minimum longitudinal acceleration
v.axmax=3;                  %[m/s2] maximum longitudinal acceleration
v.aymin=-2;                 %[m/s2] minimum lateral acceleration
v.aymax=2;                  %[m/s2] maximum lateral acceleration
v.Daxmin=-15;               %[m/s3] minimum longitudinal jerk
v.Daxmax=7.5;               %[m/s3] maximum longitudinal jerk
v.Daymin=-2.5;              %[m/s3] minimum lateral jerk
v.Daymax=2.5;               %[m/s3] maximum lateral jerk

v.maxslipangle=10*pi/180;   %[rad] max slip angle
v.horizon=100;              %[m] receeding horizon





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-03.

