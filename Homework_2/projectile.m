%Timothy Roche
%Weapon Systems

%Simulation Conditions
%--------------------------
SIM_RUNTIME = 110; %seconds
INC = 0.1; %New datapoint every x seconds
v_o = 500; %Initial velocity magnitude
diameter = 0.3; 
m = 250; %kg
%--------------------------

total_points = floor(SIM_RUNTIME/INC);
A = pi()*(diameter/2)^2;
angles = 10:10:80;
%-------------Part 1----------------
dis_x = zeros(8, total_points+1);
dis_y = zeros(8, total_points+1);
vel_x = zeros(8, total_points+1);
vel_y = zeros(8, total_points+1);
acc_x = zeros(8, total_points+1);
acc_y = zeros(8, total_points+1);
timeNoAir = zeros(8, total_points+1);
distanceFinal = zeros(1,8);
velFinal = zeros(1,8);

%Simuating No Air Resistance...
maxDistanceND = 0;
maxTimeND = 0;
%Collecting Data...
for i = 1:8
angle = i*10;
[x_x, y_x, x_v, y_v, x_a, y_a, time, timef, distancef, velf] = PM_noAir(angle, v_o, total_points, INC);
dis_x(i,:) = x_x;
dis_y(i,:) = y_x;
vel_x(i,:) = x_v;
vel_y(i,:) = y_v;
acc_x(i,:) = x_a;
acc_y(i,:) = y_a;
timeNoAir(i,:) = time;

distanceFinal(:,i) = distancef;
velFinal(:,i) = velf;
if(distancef > maxDistanceND)
    maxDistanceND = distancef;
end
if(timef > maxTimeND)
    maxTimeND = timef;
end
end

subPlotX = 2;
subPlotY = 5;

figure(1);
movegui('west');
sgtitle("Projectile Motion -- No Air Resistance");
plotData(timeNoAir, dis_x/1000,  "Distance VS Time", "Time (s)", "Distance (km)",subPlotX, subPlotY, 1, 0, maxTimeND);
plotData(timeNoAir, dis_y/1000, "Height VS Time", "Time (s)", "Height (km)", subPlotX,subPlotY,2, 0, maxTimeND)
plotData(timeNoAir, vel_x, "X-Velocity VS Time", "Time (s)", "X-Velocity (m/s)", subPlotX,subPlotY,3, 0, maxTimeND);  
plotData(timeNoAir, vel_y, "Y-Velocity VS Time", "Time (s)", "Y-Velocity (m/s)", subPlotX,subPlotY,4, 0, maxTimeND); 
plotData(timeNoAir, acc_x/9.8, "X-Acceleration VS Time", "Time (s)", "X-Acceleration (G)",subPlotX,subPlotY,5, 0, maxTimeND);
plotData(timeNoAir, acc_y/9.8, "Y-Acceleration VS Time", "Time (s)", "Y-Acceleration (G)",subPlotX,subPlotY,6, 0, maxTimeND);
plotData(angles, distanceFinal/1000, "Launch Angle Vs Distance", "Launch Angle (degrees)", "Distance (km)", subPlotX, subPlotY, 7, 10, inf);
plotData(angles, velFinal, "Launch Angle Vs Final Velocity", "Launch Angle (degrees)", "Final Velocity (m/s)", subPlotX, subPlotY, 8, 10, inf);
ylim([480, 520]); %Fixes some weird labeling issues the graph was having

%Simulating with Air Resistance modeled as kv^2
dis_x_Drag = zeros(8, total_points);
dis_y_Drag = zeros(8, total_points);
vel_x_Drag = zeros(8, total_points);
vel_y_Drag = zeros(8, total_points);
acc_x_Drag = zeros(8, total_points);
acc_y_Drag = zeros(8, total_points);
timeDrag = zeros(8, total_points);
distanceFinal_Drag = zeros(1,8);
velFinal_Drag = zeros(1,8);

maxDistanceD = 0;
maxTimeD = 0;

%Collecting Data...
for i = 1:8
angle = i*10;
[x_a_Drag, y_a_Drag, x_v_Drag, y_v_Drag, x_x_Drag, y_x_Drag, time_Drag, timef_Drag, distancef_Drag, velf_Drag] = PM_withAirResistance(m, A, v_o, angle, INC, SIM_RUNTIME);
dis_x_Drag(i,:) = x_x_Drag;
dis_y_Drag(i,:) = y_x_Drag;
vel_x_Drag(i,:) = x_v_Drag;
vel_y_Drag(i,:) = y_v_Drag;
acc_x_Drag(i,:) = x_a_Drag;
acc_y_Drag(i,:) = y_a_Drag;
timeDrag(i,:) = time_Drag;

distanceFinal_Drag(:,i) = distancef_Drag;
velFinal_Drag(:,i) = velf_Drag;
disp(timef_Drag);
if(distancef_Drag > maxDistanceD)
    maxDistanceD = distancef_Drag;
end
if(timef_Drag > maxTimeD) %Stops graphs from sitting at 0 after landing
    maxTimeD = timef_Drag;
end
end

figure(2);
movegui('east');
subPlotX = 2;
subPlotY = 5;
subplot(subPlotX,subPlotY,1);

sgtitle("Projectile Motion -- With Quadratic Air Resistance");
plotData(timeDrag, dis_x_Drag/1000,  "Distance VS Time", "Time (s)", "Distance (km)",subPlotX, subPlotY, 1,0,maxTimeD);
plotData(timeDrag, dis_y_Drag/1000, "Height VS Time", "Time (s)", "Height (km)", subPlotX,subPlotY,2,0,maxTimeD)
plotData(timeDrag, vel_x_Drag, "X-Velocity VS Time", "Time (s)", "X-Velocity (m/s)", subPlotX,subPlotY,3,0,maxTimeD);  
plotData(timeDrag, vel_y_Drag, "Y-Velocity VS Time", "Time (s)", "Y-Velocity (m/s)", subPlotX,subPlotY,4,0,maxTimeD); 
plotData(timeDrag, acc_x_Drag/9.8, "X-Acceleration VS Time", "Time (s)", "X-Acceleration (G)",subPlotX,subPlotY,5,0,maxTimeD);
plotData(timeDrag, acc_y_Drag/9.8, "Y-Acceleration VS Time", "Time (s)", "Y-Acceleration (G)",subPlotX,subPlotY,6,0,maxTimeD);
plotData(angles, distanceFinal_Drag/1000, "Launch Angle Vs Distance", "Launch Angle (degrees)", "Distance (km)", subPlotX, subPlotY, 7, 10, inf);
plotData(angles, velFinal_Drag, "Launch Angle Vs Final Velocity", "Launch Angle (degrees)", "Final Velocity (m/s)", subPlotX, subPlotY, 8, 10, inf);

%Bonus Graph
figure(3);
sgtitle("Projectile Motion -- Distance VS Height");
plotData(dis_x/1000, dis_y/1000, "No Air Resistance", "Distance (km)", "Height (km)",2,1,1, 0, (maxDistanceND/1000));
plotData(dis_x_Drag/1000, dis_y_Drag/1000, "With kv^2 Air Resistance", "Distance (km)", "Height (km)",2,1,2,0, (maxDistanceD/1000));

%----------Functions-------------

function plotData(xData, yData, name, xl, yl,subX, subY, POS,xPosMin, xPosMax)
    %Plotting Data
    subplot(subY,subX,POS);
    title(name);
    xlabel(xl);
    ylabel(yl);
    hold on
    temp = size(xData);
    ySize = temp(1); %So that this function works with different inputss
    for i = 1:ySize
        data_x = xData(i, :);
        data_y = yData(i, :);
        plot(data_x, data_y);
    end
    hold off
    xlim([xPosMin, xPosMax]);
end



   