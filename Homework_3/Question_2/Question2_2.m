%Given Conditions
diameter = 0.182;
alpha_max = 30;
h = 10000;
v = 600;
mass = 200;
g = 9.81;
G = 20;
Nc = mass*g;

%CN_Alpha Calculation
[t,p, density, mach, Q] = atmosModel(h,v);
sRef = pi()*(diameter/2)^2;
alpha_max_RAD = (alpha_max*pi())/180;
cnalpha = (G*mass*g)/(alpha_max_RAD*Q*sRef);

disp(cnalpha);

%Iterations

gamma = 10; %In degrees
alpha = 10; %In degrees

theta = alpha + gamma; %In degrees
nForce = Nc/cosd(theta);
accNorm = Nc/(cosd(theta) * mass);
alpha = nForce/(cnalpha
