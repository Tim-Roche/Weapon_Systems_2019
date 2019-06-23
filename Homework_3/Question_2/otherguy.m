%CannonBall
clear all;
close all;
colSet = {'blue', 'red', 'green', 'yellow'};

 useDrag = 1;
%Conversion Factors
km2m =1000;    %km / m
m2km = 1/km2m; %m/km
G = 9.81;      %m/sec

%set simulation initialization parameters
fpaArray = 10:10:60; %initial flight path angle
dt = 0.1; % Time step for simulation


%Turn on and off commands
gravityCompOn = 1;  %enable guidance command "1 G Up"
zeroLiftDragOn = 1; %enable flight with zero lift drag
offset = 1;         %color offset for different lines
iplot = 1;          %Plot results for individual trajectories

%Basic iniilization
colIndx = zeroLiftDragOn + offset;
run = 0; %run number

%physical definitions of missilie
h0       = 0;                 % m   initial hieght
sref     = pi*(.182/2)^2;     % m^2 reference area
alim     = 30 * pi/180;       % rad, angle limit
Amax_g   = 20;                % G
Amax_alt = 10000;             % km
thrust   = 0;                 % G
glim     = 999;               % G
mass = 200;                   % kg mass
xd0 = 600;                    % m/s Intitial speed 

% Solve for CN_alpha
%Nmax = mass*Amax_g*G = (CN_alpha)*alim*sref*Q
a = atmosphericModel(Amax_alt, norm(xd0));
Nmax = (Amax_g*G)*mass;
CNmax = Nmax / (sref*a.Q);
CN_alpha = CNmax/alim;
%From worksspace, CN_alpha = 38.5968

%Axial Coefficients
if useDrag > 0
    axiCoeff.CA0 = 0.040;
    axiCoeff.CA1 = 0.020;
    axiCoeff.CA2 = 0.060;
else
    axiCoeff.CA0 = 0;
    axiCoeff.CA1 = 0;
    axiCoeff.CA2 = 0;
end

%Loop through Flight Path Angle

for fpa = fpaArray
    
    %increment run counter
    run = run + 1;
    
    %Define initial run conditions
    x = [0;h0];
    xd = [cosd(fpa); sind(fpa)] * xd0;
    xhat = [x; xd];
    t = 0;
    NacG = [0;0];
    ncPitchLast = 0;
    gamma = asind(xd(2)/norm(xd));
    theta = gamma;
    
    %initilize incremental counter
    cnt = 0;
    
    %loop through aerodynamics for one trajector until projectile hits
    %ground
    while (x(2) >= 0)
        gamma = asin(xd(2)/norm(xd));
        if (t > 1)
            NacG = [0;1]*gravityCompOn;
        end;
        
        %compute atmospheric properties at current time and postion
        atmo = atmosphericModel(x(2), norm(xd));
        
        %compute Net accleration from Normal Force
        
        %compute component acc normal to body centerline
        ncBody = cos(theta)*NacG(2) - sin(theta)*NacG(1);
        
          % Provide a G-limit such that the missile can never attempt to
          % achieve more than glim G. The value is currently set to an
          % unreasonable 999.
          ncLimBody = min([ncBody, glim]);
        
           % Compute max angle of attack and limit to the max angle of attack, 
           % alim. 
           cbar = atmo.Q * sref/(mass*G);
           NcMax = ((CN_alpha)*(alim))*cbar; 
           alpha = (alim) * ncLimBody/(NcMax);
        
             % If angle of attack is greater than the AoA limit, clamp it and
             % compute the appropriatre normal acceleration, nz
             alphaAch = min([alpha, alim]);
             alphaRatio = 0; 
             if (alpha>0)  
                 alphaRatio = alphaAch/alpha;
             end
             alpha = alphaAch; 
             nz = ncLimBody * alphaRatio;
             
             CN = (nz)/cbar; 
             CDI = CN*alpha; 
             iDrag = nz*sin(alpha); 
        
             % Compute new body axis orientation 
             theta = gamma + alpha;
             % Save the achieved pitch for the next pass through
             ncPitchLast = nz;

              if (atmo.mach < 0.5)
                 % CA is a constant
                 CA   = axiCoeff.CA0; 
              elseif (atmo.mach < 1.0)
                  % Value of CA at Mach 1 
                  CAM1  = axiCoeff.CA2 + axiCoeff.CA1;
                  % Value of CA at Mach 0.5 
                  CAM05 = axiCoeff.CA0;
                  % Delta of CA at Mach 1 and CA at Mach 0.5 
                  DCA  = CAM1 - CAM05;
                  % Interpolate based upon Mach 0.5 and Mach 1.0  
                  CA   = axiCoeff.CA0 + (DCA/0.5)*(atmo.mach-0.5);
              else
                  % CA is a described by equation
                  CA   = (axiCoeff.CA1 + axiCoeff.CA2./(atmo.mach));
              end;
              if (zeroLiftDragOn == 0)
                  CA = 0;
              end;
              
              % Convert the axial force coefficient to axial acceleration
              axDragAcc = (CA)*cbar;  % G 
              thrustAcc = 0;          % G
              axAcc = (thrustAcc - axDragAcc);
              
               % Compute Net Acceleration on the Projectile 
               netAcc = zeros(2,1);
               netAcc(1) =  axAcc*cos(theta) - nz*sin(theta);
               netAcc(2) =  axAcc*sin(theta) + nz*cos(theta);

               % integrate from time t to time t + dt
               t = t + dt;
               cnt = cnt + 1;
               
               %compute net acc of projectile
               Gvect = [0;-1];
               xacc = netAcc + Gvect;
               xdd = xacc * G;
               
               %integrate kinematic states of projectile
               x = x + dt*xd + (1/2)*xdd*dt^2;
               xd = xd + dt*xdd;
               
               %store outputs to plot
               output(run).t(cnt)      = t;
               output(run).x1(cnt)     = x(1) .* m2km;
               output(run).x2(cnt)     = x(2) .* m2km;
               output(run).x(cnt)       = norm(x) .* m2km;
               output(run).xd1(:,cnt)   = xd(1);
               output(run).xd2(:,cnt)   = xd(2);
               output(run).xd(:,cnt)    = norm(xd); 
               output(run).xdd1(:,cnt)  = xdd(1) ./ G;
               output(run).xdd2(:,cnt)  = xdd(2) ./ G;
               output(run).xdd(:,cnt)   = norm(xdd) ./ G;
               output(run).gamma(cnt)   = gamma*180/pi;   
               output(run).alpha(cnt)   = alpha*180/pi; 
               output(run).theta(cnt)   = theta*180/pi; 
               output(run).iDrag(cnt)   = iDrag;
               output(run).axDragAcc(cnt)  = axDragAcc;  
               output(run).nz(cnt)         = nz;
               
        end;
        
        %stroe End tracjector info
        finalSpeed(run) = norm(xd);
        finalDistance(run) = norm(x) * m2km;
end;
        
%Plot results for each run.             
fpaString =  cellstr(num2str([fpaArray]'));             
leg = strcat('\gamma_0 = ', fpaString);             
 
figure(1); hold on; grid on;
title('Trajectry');
[hPlot1] = PlotDSX(output, 'x1', 'x2');
legend(leg);
xlabel('km')
ylabel('km')

%Plots Position
figure(2);
subplot(3,1,1); hold on; grid on;
title('x pos');
[hPlot2_1] = PlotDSX(output, 't', 'x1');
ylabel('km');
subplot(3,1,2); hold on; grid on;
title('y pos');
[hPlot2_2] = PlotDSX(output, 't','x2');
ylabel('km');
subplot(3,1,3); hold on; grid on;
title('Range total');
[hPlot2_3] = PlotDSX(output, 't','x');
ylabel('km');

%Plots velocity
figure(3);
subplot(3,1,1); hold on; grid on;
title('x velocity');
[hPlot3_1] = PlotDSX(output, 't', 'xd1');
ylabel('m/s');
subplot(3,1,2); hold on; grid on;
title('y velocity');
[hPlot3_2] = PlotDSX(output, 't', 'xd2');
ylabel('m/s');
subplot(3,1,3); hold on; grid on;
title('Speed');
[hPlot3_3] = PlotDSX(output, 't', 'xd');
ylabel('m/s');

%Plots Acceleration
figure(4);
subplot(3,1,1); hold on; grid on;
title('X acc');
[hPlot3_1] = PlotDSX(output, 't', 'xdd1');
ylabel('G');
subplot(3,1,2); hold on; grid on;
title('Y acc');
[hPlot3_2] = PlotDSX(output, 't', 'xdd2');
ylabel('G');
subplot(3,1,3); hold on; grid on;
title('Total acceleration');
[hPlot3_3] = PlotDSX(output, 't', 'xdd');
ylabel('G');

%Final distance and speed plots per flgith angle
figure(5); hold on; grid on;
plot(fpaArray, finalDistance)%, 'b', 'color');%, rgb(colSet{colIndx}));
stem(fpaArray, finalDistance)%, 'b', 'color');% rgb(colSet{colIndx}));
title('Final Distance Achieved');
xlabel('initial Flight Path Angle (deg) ');
ylabel('km');

figure(6); hold on; grid on;
plot(fpaArray, finalSpeed)%, 'b', 'color'); %, rgb(colSet{colIndx}));
stem(fpaArray, finalSpeed)%, 'c', 'color'); %, rgb(colSet{colIndx}));
title('Final Speed Achieved');
xlabel('initial flight path angle (deg)');
ylabel('m/s');
