incTime = 0.1;
RUNTIME = 30; %Seconds

VBO = 1000; %m/s
mBO = 500; %Mass at Burn out
mF = 300; %Mass of fuel
mo = mBO+mF; %Initial Mass
g = -9.8; %m/s^2
G = 15;
Isp = VBO/(g*G*log((500+300)/500)); %Compute the ISP required 
Ve =  G*g*Isp; %Compute Exit Velocity
mdot = G*(mBO/Isp);
F = Isp*mdot*g;

[wSeries, aSeries, timeSeries, tBO] = weightAndA(mdot, mo, mBO, F, incTime, RUNTIME);
plotData(timeSeries, wSeries, "Weight VS Time", "Time (s)", "Weight (kg)", 1,2, 1, -inf, inf);
ylim([0 inf]);

plotData(timeSeries, aSeries/g, "Acc. VS Time", "Time (s)", "Acceleration (G)", 1,2, 2, -inf, inf);

function [wSeries, aSeries, timeSeries, tBO] = weightAndA(mdot, mo, mBO, F, incTime, endTime)
    totalPoints = floor(endTime/incTime);
    wSeries = zeros(1, totalPoints);
    aSeries = zeros(1, totalPoints);
    timeSeries = zeros(1, totalPoints);
    wo = mo*9.8; %Converting mass to weight
    wdot = mdot*9.8;
    for i = 1:totalPoints
        t = i*incTime;
        wSeries(i) = wdot*t + wo; %y = mx + b
        aSeries(i) = F /(wSeries(i)); %Take integral to get v no grav.
        if(wSeries(i) < mBO)
            wSeries(i) = mBO;
            aSeries(i) = 0;
        end
        timeSeries(i) = t;
    end
    tBO = (mBO-wo)/wdot;
end