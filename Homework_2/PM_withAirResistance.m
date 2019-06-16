function [a_x, a_y, v_x, v_y, x_x, x_y, timeArray] = PM_withAirResistance(m, A, v_o, angle, deltaT, simRunTime)
%Assuming Quadratic Air Resistance
g = 9.8;
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
for i = 2:totalPoints %Iterating using Euler
    timeArray(i) = timeArray(i-1)+deltaT;
    [t_x,p_x, density_x, mach_x, Q_x] = atmosModel_1(x_y(i-1), v_x(i-1));
    [t_y,p_y, density_y, mach_y, Q_y] = atmosModel_1(x_y(i-1), v_y(i-1));
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
end
function [t,p, density, mach, Q] = atmosModel_1(h, s)
    t=0; %temperature in cel
    p=0; %pressure
    Vsound_FPS = 49*sqrt(celToRankine(t)); %speed of sound in FPS
    mach = s/feetPS_to_metersPS(Vsound_FPS);
    if(h > 25000)
        t = -131.21 + 0.00299*h;
        p = 2.488 * ((t+273.1)/216.6) ^(-11.388);
        
    elseif((h > 11000)&&(h<25000))
        t=-56.46;
        p=22.65*exp(1.73-0.000157*h);
   
    else
        t = 15.04 - 0.00649*h;
        p = 101.29 * ((t+273.1)/288.08)^(5.256);
    end
    
    density = p /(.2869 * (t + 273.1));
    Q = (density/2)*(s^2); %Dynamic Pressure
    function r = celToRankine(c)
        r = (c + 273.15) * (9/5);
    end

    function metersPerSecond = feetPS_to_metersPS(feetPerSecond)
        metersPerSecond = feetPerSecond*0.3048;
    end
end
end