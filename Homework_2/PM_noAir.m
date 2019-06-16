%Timothy Roche
%Weapon Systems
function [x_x, y_x, x_v, y_v, x_a, y_a, time_array, timef, distancef, velf] = PM_noAir(angle, v_o, total_points, INC)
g = -9.8;

v_ox = v_o*cosd(angle);
v_oy = v_o*sind(angle);

x_x = zeros(1, total_points);
y_x = zeros(1, total_points);
y_v = zeros(1, total_points);
x_v = zeros(1, total_points);
x_a = zeros(1, total_points); 
y_a = zeros(1, total_points);

time_array = zeros(1, total_points);

for dataPoint = 1:total_points+1 %Simulating all datapoints
    time = (dataPoint-1) * INC;
    time_array(dataPoint) = time;
    x_x(dataPoint) = bufferAboveZero(v_ox*time); %X Pos
    y_x(dataPoint) = bufferAboveZero((1/2)*g*time^2 + v_oy*time); %Y Pos
    x_v(dataPoint) = v_ox; %X Velocity 
    y_v(dataPoint) = g*time+v_oy; %Y Velocity
    x_a(dataPoint) = 0;
    y_a(dataPoint) = g; 
end
 
%Final time velocity and distance calculations
timef = (-v_oy/g)*2;
velf = sqrt(v_ox^2 + (g*timef+v_oy)^2);
distancef = v_ox*timef;

end