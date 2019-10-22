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
 x = z;

[pks,locs] = findpeaks(x,t,'MinPeakHeight',20,'MinPeakDistance',2);
t_interval = mean(locs(2:end) - locs(1:end-1));
figure(), plot(locs,pks,'r', t , x,'b')



