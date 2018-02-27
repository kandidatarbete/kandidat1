%% plot accelrations
figure; 
subplot(4,1,1); hold on
plot(acc_c1.time,acc_c1.signals.values,'Linewidth',1.5)
% plot(acc_c1_act.time,acc_c1_act.signals.values)
plot(acc_c2.time,acc_c2.signals.values,'Linewidth',1.5)
plot(acc_c3.time,acc_c3.signals.values,'Linewidth',1.5)

% legend('Car 1','Car 2','Car 3')
xlim([15, 28]);
% title('Acceleration')
ylabel({'\bf Acceleration','[m/s^2]'})

subplot(4,1,2); hold on;
plot(vx_c1.time,vx_c1.signals.values*3.6,'Linewidth',1.5);
plot(vx_c2.time,vx_c2.signals.values*3.6,'Linewidth',1.5);
plot(vx_c3.time,vx_c3.signals.values*3.6,'Linewidth',1.5);

%legend('Car 1','Car 2','Car 3')
xlim([15, 28]);
% title('Velocity')
ylabel({'\bf Velocity','[Km/h]'})

subplot(4,1,3); hold on;
plot(distance_c1.time,distance_c1.signals.values-distance_c1.signals.values(1),'Linewidth',1.5);
plot(distance_c2.time,distance_c2.signals.values-distance_c2.signals.values(1),'Linewidth',1.5);
plot(distance_c3.time,distance_c3.signals.values-distance_c3.signals.values(1),'Linewidth',1.5);

%  find(dist2int_c1.signals.values == 0)
% leav1
plot(distance_c1.time(find(dist2int_c1.signals.values <= 0,1))...
    ,distance_c1.signals.values(find(dist2int_c1.signals.values <= 0,1))-distance_c1.signals.values(1),'^','Color',[0 0.4470 0.7410],'Linewidth',3);
plot(distance_c1.time(find(dist2int_c1.signals.values <= -14,1))...
    ,distance_c1.signals.values(find(dist2int_c1.signals.values <= -14,1))-distance_c1.signals.values(1),'v','Color',[0 0.4470 0.7410],'Linewidth',3);

plot(distance_c2.time(find(dist2int_c2.signals.values <= 0,1))...
    ,distance_c2.signals.values(find(dist2int_c2.signals.values <= 0,1))-distance_c2.signals.values(1),'^','Color',[0.8500 0.3250 0.0980],'Linewidth',3);
plot(distance_c2.time(find(dist2int_c2.signals.values <= -14,1))...
    ,distance_c2.signals.values(find(dist2int_c2.signals.values <= -14,1))-distance_c2.signals.values(1),'v','Color',[0.8500 0.3250 0.0980],'Linewidth',3);

plot(distance_c3.time(find(dist2int_c3.signals.values <= 0,1))...
    ,distance_c3.signals.values(find(dist2int_c3.signals.values <= 0,1))-distance_c3.signals.values(1),'^','Color',[0.9290 0.6940 0.1250],'Linewidth',3);
plot(distance_c3.time(find(dist2int_c3.signals.values <= -14,1))...
    ,distance_c3.signals.values(find(dist2int_c3.signals.values <= -14,1))-distance_c3.signals.values(1),'v','Color',[0.9290 0.6940 0.1250],'Linewidth',3);

xlim([15, 28]);
ylim([200 370])
% title('Position')
ylabel({'\bf Position','[m]'})
legend('Car 1','Car 2','Car 3')




subplot(4,1,4);
plot(comp_time.time,comp_time.signals.values,'Linewidth',1.5)
xlim([15, 28]);
% title('Computation Time')
ylabel({'\bf Computation Time','[s]'})

xlabel('\bf Time [s]')

%%
figure; hold on;
plot(exflag.time,exflag.signals.values)

%%
ds = 4;
ang = 0:0.01:2*pi;

figure; hold on; axis equal; grid on;
A = [0 1; 0 0];
lambda = eig(A);
%plot(cos(ang)-1,sin(ang),'Linewidth',1.5);
fill(cos(ang),sin(ang),[0.9 0.9 0.9],'EdgeColor','k');
plot(ds*real(lambda)+ones(2,1),ds*imag(lambda),'x','Linewidth',3);
ylabel({'\bf Imaginary'})
xlabel({'\bf Real'})
title('Eignevalues Of The Discritized Model')
axis([-1.2 1.2 -1.2 1.2])

%%

figure; hold on;
plot(vx_c1.time,vx_c1.signals.values);
plot(vx_c2.time,vx_c2.signals.values);
plot(vx_c3.time,vx_c3.signals.values)
%% x vector
dist2int = [dist2int_c1.signals.values, dist2int_c2.signals.values, dist3int_c1.signals.values];


for k = 200:1:length(xvec.time) % samples
N = contR+critZ+dist2Int(k,:);
N(N < 0 | N > No) = 0; % The car have to be inside the control horizon 
    
    
for j = 1:3  % cars
    if N(j) > 0
        vel(:,k,j) = 1/xvec.signals.values(sum(N(1:j-1))*4+2:2:N(j)*2,1,k);
        time(:,k,j) = xvec.signals.values(sum(N(1:j-1))*4+1:2:N(j)*2,1,k);
        if dist2int(k,j)>0
            t_ent() = xvec.signals.values(sum(N(2:j-1))*4 + dist2int(k,j),1,k);
        else 
            t_ent = 0;
        end
        if dist2int(k,j)+param.critZone > 0
            t_leav = xvec.signals.values(sum(N(2:j-1))*4 + dist2int(k,j)+ param.critZone,1,k);
        else
            t_leav = 0;
        end
    else
        vel(k,j) = 0;
        time(k,j) = 0;
        t_ent = 0;
        t_leav = 0;
    end
    
    
end



end
