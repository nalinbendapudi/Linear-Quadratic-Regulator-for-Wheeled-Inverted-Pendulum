% Code used to calculate time period needed to calculate moments of inertia

clear all
% m = csvread('gyroX3.csv');
% m = csvread('gyroY3.csv');
m = csvread('gyroZ3.csv');
t = m(:,1);
t = (t-t(1))/1000000;
x = m(:,3);
y = m(:,2);
z = m(:,4);
 
axis = z;

[pks,locs] = findpeaks(axis,t,'MinPeakHeight',20,'MinPeakDistance',2);
t_interval_z = mean(locs(2:end) - locs(1:end-1));
figure(), plot(locs,pks,'r', t , axis,'b')

m = csvread('gyroY3.csv');
t = m(:,1);
t = (t-t(1))/1000000;
x = m(:,3);
y = m(:,2);
z = m(:,4);
 
axis = y;

[pks,locs] = findpeaks(axis,t,'MinPeakHeight',20,'MinPeakDistance',2);
t_interval_y = mean(locs(2:end) - locs(1:end-1));
figure(), plot(locs,pks,'r', t , axis,'b')

m = csvread('gyroX3.csv');
t = m(:,1);
t = (t-t(1))/1000000;
x = m(:,3);
y = m(:,2);
z = m(:,4);
 
axis = x;

[pks,locs] = findpeaks(axis,t,'MinPeakHeight',20,'MinPeakDistance',2);
t_interval_x = mean(locs(2:end) - locs(1:end-1));
figure(), plot(locs,pks,'r', t , axis,'b')

m0 =1114.2/1000; % in kg
g = 9.81; % m/s^2
dx = 127/1000; % in m
dy = 74.69/1000; % in m
dz = 155.3/1000; % in m
L = 1470/1000; % in m

J_all = (m0*g)/(16 * pi^2*L);
Jxx = J_all * dx^2 * t_interval_x^2;
Jyy = J_all * dy^2 * t_interval_y^2;
Jzz = J_all * dz^2 * t_interval_z^2;


