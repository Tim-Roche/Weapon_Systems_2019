%Timothy Roche
%Weapon Systems
function [a_x, a_y, v_x, v_y, x_x, x_y, timeArray, timef, distancef, velf] = PM_withAirResistance(m, A, v_o, angle, deltaT, simRunTime)
%Assuming Quadratic Air Resistance
g = 9.8;

%Initializing Final Trackers (Final time, final velocity, ...)
timef = 0;
velf = 0;
distancef = 0;
totalPoints = floor(simRunTime/deltaT);
timeArray = zeros(1, totalPoints);
v_x = zeros(1, totalPoints);
v_y = zeros(1, totalPoints);
a_x = zeros(1, totalPoints);
a_y = zeros(1, totalPoints);
x_x = zeros(1, totalPoints);
x_y = zeros(1, totalPoints);

%Setting Initial Conditions
v_x(1) = v_o*cosd(angle);
v_y(1) = v_o*sind(angle);
x_x(1) = 0;
x_y(1) = 0;
timeArray(1) = 0;
lock = false;
for i = 2:totalPoints %Iterating using Euler
    timeArray(i) = timeArray(i-1)+deltaT;
    [t_x,p_x, density_x, mach_x, Q_x] = atmosModel(x_y(i-1), v_x(i-1));
    [t_y,p_y, density_y, mach_y, Q_y] = atmosModel(x_y(i-1), v_y(i-1));
    k_x = (1/2)*CA_model(mach_x)*density_x*A;
    k_y = (1/2)*CA_model(mach_y)*density_y*A;
    vMag = sqrt(v_x(i-1)^2+v_y(i-1)^2);
    a_x(i-1) = (-k_x*vMag*v_x(i-1))/m;
    a_y(i-1) = -g - (k_y*vMag*v_y(i-1))/m;
    v_x(i) = v_x(i-1) + a_x(i-1)*deltaT;
    v_y(i) = v_y(i-1) + a_y(i-1)*deltaT;
    x_x(i) = x_x(i-1) + v_x(i-1)*deltaT;
    x_y(i) = bufferAboveZero(x_y(i-1) + v_y(i-1)*deltaT); 
    %BufferAboveZero Prevents projectile from going under ground!
    if((x_y(i) <= 0)&&(~lock))
        lock = true; 
        %lock Prevents statement from running again once we have an answer
        timef = timeArray(i);
        velf = sqrt((v_x(i))^2 + (v_y(i)^2));
        distancef = x_x(i);
        disp("--------");
        disp(angle);
        disp(distancef);
        disp(velf);
    end
end
end