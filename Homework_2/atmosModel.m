% ------------------- atmosModel.m--------------------
%%%%%%%%%%%%%%%
%Atmospheric Model
%
%Input Units 
%h = height (meters)
%s = speed (m/s)
%
%Output Units
%t = temp. (Celsius)
%p = pressure (KPa)
%density (kg/m^3)
%mach (unitless)
%Q  = Dynamic Pressure (Pascals)
%%%%%%%%%%%%%%%

function [t,p, density, mach, Q] = atmosModel(h, s)
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
% ------------------------------------------------
    
        