%% Initial Script

numCars = 3;




% Parameters
% ---------------------
param.weights(1) = 1;   % wightening cost fcn 1   (velocity)
param.weights(2) = 1100;   % wightening cost fcn 2   (acceleration)
param.weights(3) = 23; % wightening cost fcn 3      (jerk)
param.weights(4) = 10000; % wightening cost fcn 3  (time gap)
param.finalWeights(1) = 4000000; %2000;
param.finalWeights(2) = 200000; %1000;
param.finalWeights(3) = 100; % 1000;

param.sampling = 1;
param.contrRadius = 60;
param.critZone = 14;
param.timediff = 0.6;
param.ds = 4;
param.T = 0.05; % Sampling time

% System Matrices
%----------------------
mat.A = [1 param.ds ; 0 1 ]; 
mat.B = [0 ; param.ds];
mat.C = [0, 1];
mat.Qv = [0 0; 0 1];
mat.Qt = [1 0; 0 0];


% Vehicle data
% ---------------------
cars1.refVel = 47/3.6;
cars2.refVel = 48/3.6;
cars3.refVel = 50/3.6; 

cars1.meanVel = mean(cars1.refVel);
cars2.meanVel = mean(cars2.refVel);
cars3.meanVel = mean(cars3.refVel);

cars1.dist2Int = 76;
cars2.dist2Int = 78;
cars3.dist2Int = 80;

cars1.initDist = cars1.dist2Int;
cars2.initDist = cars2.dist2Int;
cars3.initDist = cars3.dist2Int;


% Constraints
% ---------------------
const.vmax = 90/3.6;
const.vmin = 30/3.6;
const.amax = 3;
const.amin = -3;
