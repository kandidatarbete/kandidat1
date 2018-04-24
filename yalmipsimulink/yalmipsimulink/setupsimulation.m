% System
Plant = ss(tf(1,[1 0 0]));
A = Plant.A;
B = Plant.B;
C = Plant.C;
D = Plant.D;

% Global sampling-time
Ts = 0.1;

% Initial state for simulation
x0 = [1;1];