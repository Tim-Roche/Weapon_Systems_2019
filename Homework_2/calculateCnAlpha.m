%Timothy Roche
%Weapon Systems
mass = 350;
diameter = 0.5;
alpha = 25;
h = 15000;
v = 500;
mass = 350;

[t,p, density, mach, Q] = atmosModel(h,v);

G = 20;
sRef = pi()*(diameter/2)^2;
Cnalpha = (G*mass*9.8)/(alpha*Q*sRef);
disp(Cnalpha);

[t,p,density, mach, Q] = atmosModel(25002, 0);

function r = celToRankine(c)
    r = (c + 273.15) * (9/5);
end

function metersPerSecond = feetPS_to_metersPS(feetPerSecond)
    metersPerSecond = feetPerSecond*0.3048;
end
