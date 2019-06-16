SIM_RUNTIME = 200; %seconds
INC = 0.1;
total_points = floor(SIM_RUNTIME/INC);
v_o = 500;
deltaT = 0.5;
diameter = 0.3; 
m = 250;

%-------------Part 1----------------
dis_x = zeros(8, total_points+1);
dis_y = zeros(8, total_points+1);
vel_x = zeros(8, total_points+1);
vel_y = zeros(8, total_points+1);
acc_x = zeros(8, total_points+1);
acc_y = zeros(8, total_points+1);
timeNoAir = zeros(8, total_points+1);

%Simuating No Air Resistance...
for i = 1:8
angle = i*10;
[x_x, y_x, x_v, y_v, x_a, y_a, time] = PM_noAir(angle, v_o, total_points, INC);
dis_x(i,:) = x_x;
dis_y(i,:) = y_x;
vel_x(i,:) = x_v;
vel_y(i,:) = y_v;
acc_x(i,:) = x_a;
acc_y(i,:) = y_a;
timeNoAir(i,:) = time;
end

subPlotX = 2;
subPlotY = 3;

figure(1);
sgtitle("Projectile Motion -- No Air Resistance");
plotData(timeNoAir, dis_x/1000,  "Distance VS Time", "Time (s)", "Distance (km)",subPlotX, subPlotY, 1);
plotData(timeNoAir, dis_y/1000, "Height VS Time", "Time (s)", "Height (km)", subPlotX,subPlotY,2)
plotData(timeNoAir, vel_x, "X-Velocity VS Time", "Time (s)", "X-Velocity (m/s)", subPlotX,subPlotY,3);  
plotData(timeNoAir, vel_y, "Y-Velocity VS Time", "Time (s)", "Y-Velocity (m/s)", subPlotX,subPlotY,4); 
plotData(timeNoAir, acc_x, "X-Acceleration VS Time", "Time (s)", "X-Acceleration (m/s^2)",subPlotX,subPlotY,5);
plotData(timeNoAir, acc_y/9.8, "Y-Acceleration VS Time", "Time (s)", "Y-Acceleration (G)",subPlotX,subPlotY,6);


%-------------Part 3----------------
dis_x_Drag = zeros(8, total_points);
dis_y_Drag = zeros(8, total_points);
vel_x_Drag = zeros(8, total_points);
vel_y_Drag = zeros(8, total_points);
acc_x_Drag = zeros(8, total_points);
acc_y_Drag = zeros(8, total_points);
timeDrag = zeros(8, total_points);

figure(2);
subPlotX = 2;
subPlotY = 3;
A = pi()*(diameter/2)^2;
subplot(subPlotX,subPlotY,1);

for i = 1:8
angle = i*10;
[x_a_Drag, y_a_Drag, x_v_Drag, y_v_Drag, x_x_Drag, y_x_Drag, time_Drag] = PM_withAirResistance(m, A, v_o, angle, INC, SIM_RUNTIME);
dis_x_Drag(i,:) = x_x_Drag;
dis_y_Drag(i,:) = y_x_Drag;
vel_x_Drag(i,:) = x_v_Drag;
vel_y_Drag(i,:) = y_v_Drag;
acc_x_Drag(i,:) = x_a_Drag;
acc_y_Drag(i,:) = y_a_Drag;
timeDrag(i,:) = time_Drag;
end

sgtitle("Projectile Motion -- With Quadratic Air Resistance");
plotData(timeDrag, dis_x_Drag/1000,  "Distance VS Time", "Time (s)", "Distance (km)",subPlotX, subPlotY, 1);
plotData(timeDrag, dis_y_Drag/1000, "Height VS Time", "Time (s)", "Height (km)", subPlotX,subPlotY,2)
plotData(timeDrag, vel_x_Drag, "X-Velocity VS Time", "Time (s)", "X-Velocity (m/s)", subPlotX,subPlotY,3);  
plotData(timeDrag, vel_y_Drag, "Y-Velocity VS Time", "Time (s)", "Y-Velocity (m/s)", subPlotX,subPlotY,4); 
plotData(timeDrag, acc_x_Drag, "X-Acceleration VS Time", "Time (s)", "X-Acceleration (m/s^2)",subPlotX,subPlotY,5);
plotData(timeDrag, acc_y_Drag/9.8, "Y-Acceleration VS Time", "Time (s)", "Y-Acceleration (G)",subPlotX,subPlotY,6);

figure(3);
sgtitle("Projectile Motion -- Distance VS Height");
plotData(dis_x/1000, dis_y/1000, "No Air Resistance", "Distance (km)", "Height (km)",2,1,1);
plotData(dis_x_Drag/1000, dis_y_Drag/1000, "With kv^2 Air Resistance", "Distance (km)", "Height (km)",2,1,2);
%----------Part 2-------------
%figure(4);
%final_vels = [];
%final_dists = [];
%angles = 10:10:80;
%for angle = 10:10:80
%    [final_vel, final_distance] = getFinalVelocityAndFinalDistance(angle, v_o)
%    final_vels = [final_vels, final_vel];
%    final_dists = [final_dists, final_distance];
%end

%subplot(2,5,4);
%plot(angles, final_vels);
%title("Lauch Angles VS Vo");
%xlabel("Launch Angle");
%ylabel("Vf");
%ylim([450 550]);

%subplot(2,5,5);
%plot(angles, final_dists);
%title("Launch Angles VS Xf");
%xlabel("Launch Angle");
%ylabel("Xf");

%----------Functions-------------

function [final_vel, final_distance] = getFinalVelocityAndFinalDistance(angle, v_o)
%Get Final Velocity
g = -9.8
v_ox = v_o*cosd(angle);
v_oy = v_o*sind(angle);

final_time = (-v_oy/g)*2;
final_vel = sqrt(v_ox^2 + (g*final_time+v_oy)^2);
final_distance = v_ox*final_time;
end

function plotData(xData, yData, name, xl, yl,subX, subY, POS)
%Plotting Data
subplot(subY,subX,POS);
title(name);
xlabel(xl);
ylabel(yl);
hold on
for i = 1:8
    data_x = xData(i, :);
    data_y = yData(i, :);
    plot(data_x, data_y);
end
hold off
end



   