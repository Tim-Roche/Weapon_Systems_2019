%Timothy Roche
%Final Project Weapon Systems
rng('default');
SIM_TIME = 50;
dt = 2;
TOTAL_POINTS = int64(SIM_TIME/dt) + 1;

%Initial Condition
P_0 = 0; %Initial Posistion
V_0 = 200; %Initial Velocity
inits = [P_0; V_0];

%Number of Monte Carlos
carlos = 500;

figure(1);
movegui('west');
acc = NaN; %Truth acceleration is white noise
[posP, velP, deltaX_c, deltaV_c, mPEx, mPEv, varX, varV, time] = Kf_MC(acc,SIM_TIME,dt,inits,TOTAL_POINTS,carlos);
graphName = "1D Kalman Filter -- a = White Noise";
plotToGraph(graphName, posP, velP, deltaX_c, deltaV_c, mPEx, mPEv, varX, varV, time, carlos);

figure(2);
movegui('east');
acc = 15; %Truth acceleration is 15m/s^2
[posP_acc, velP_acc, deltaX_c_acc, deltaV_c_acc, mPEx_acc, mPEv_acc, varX_acc, varV_acc, time] = Kf_MC(acc,SIM_TIME,dt,inits,TOTAL_POINTS,carlos);
graphName_a = "1D Kalman Filter -- a = 15m/s^2";
plotToGraph(graphName_a, posP_acc, velP_acc, deltaX_c_acc, deltaV_c_acc, mPEx_acc, mPEv_acc, varX_acc, varV_acc, time, carlos);


function [posP, velP, deltaX_c, deltaV_c, mPEx, mPEv, varX, varV, time] = Kf_MC(acc,SIM_TIME,dt,inits,TOTAL_POINTS,carlos)
    %Kalman Filter + 500 rounds of Monte Carlo
    %posP_c = zeros(TOTAL_POINTS,carlos);
    %posV_c = zeros(TOTAL_POINTS,carlos);
    deltaX_c = zeros(carlos,TOTAL_POINTS);
    deltaV_c = zeros(carlos,TOTAL_POINTS);
    
    deltaX = zeros(1, TOTAL_POINTS);
    deltaV = zeros(1, TOTAL_POINTS);   
    
    posP = zeros(1,TOTAL_POINTS);
    velP = zeros(1,TOTAL_POINTS); 
    for c = 1:carlos
        [x, p, truth, time] = kf(acc,SIM_TIME,dt,inits,TOTAL_POINTS);
        for i = 1:TOTAL_POINTS
            p2x2 = p(:,pull2x2byIndex(i));
            posP(i) = p2x2(1,1);
            velP(i) = p2x2(2,2);
            deltaX(i) = x(1,i) - truth(1, i);
            deltaV(i) = x(2,i) - truth(2, i);
        end
        %posP_c(:, i) = posP;
        %posV_c(:, i) = velP;
        deltaX_c(c, :) = deltaX;
        deltaV_c(c, :) = deltaV;
    end
    mPEx = mean(deltaX_c);
    mPEv = mean(deltaV_c);
    %for i = 1:carlos
    %    mPEx(i) = mean(deltaMeanX(1:i));
    %    mPEv(i) = mean(deltaMeanV(1:i));
    %end
    
    varX = var(deltaX_c);
    varV = var(deltaV_c);
    
    deltaX_c = reshape(deltaX_c.', 1, []);
    deltaV_c = reshape(deltaV_c.', 1, []);
end

%----------Functions-------------
function [x, p, truth, time] = kf(acc,SIM_TIME,dt,inits,TOTAL_POINTS)
time = 0:dt:SIM_TIME;

%Defining Key Matrixes
x = zeros(2, TOTAL_POINTS);
p = zeros(2,TOTAL_POINTS*2);

%Matrixes
A = [1, dt; 0, 1]; %Transistion Matrix
C = [1,0;0,1];

%"Conversion" Matrix
H = [1,0];%;1,0];

%Measurement Errors
R_true = 900;
R = 0.95*R_true; 

%Noise
W_true = 12;
W = 0.9*W_true;

%Initial Values
x(:,1) = inits; %x_prediction
p(:,pull2x2byIndex(1)) = [1, (1/dt); (1/dt), (2/dt)^2]*R_true; %Process Uncertanty

truth = zeros(2, TOTAL_POINTS);
truth(:, 1) = inits;

for i = 2:TOTAL_POINTS
    B = [(1/2)*dt^2; dt]; %Control Matrix for accel.
    
    %Find true posistion
    if(isnan(acc)) %Check to see if this is Part 1 or Part 1.1
        acc = normrnd(0, sqrt(W_true)); %Make Acc white noise instead
    end
    truth(:,i) = A*truth(:,i-1) + B*acc;
    %Get New Measured Value
    noiseX = normrnd(0, sqrt(R_true));
    measuredX = truth(1,i) + noiseX;%C*Y_m + z; 
    z = measuredX;

    %Process Noise
    Q = [(1/4)*dt^4, (1/2)*dt^3; (1/2)*dt^3, dt^2]*W;
   
    %New Predicted State
    x_pC = A*x(:, i-1); %Assuming no acceleration or error
    p_pC = A*p(:, pull2x2byIndex(i-1))*A.' + Q; %Assuming no error
    
    %Gain Computation
    KG = (p_pC*H.')/(H*p_pC*H.' + R);
    
    %Calculate Current State
    x(:, i) = x_pC + KG*(z - H*x_pC);
    p(:, pull2x2byIndex(i)) = (eye(2) - KG*H)*p_pC; %(eye(2) - KG*H)*p_pC*(eye(2)-KG*H).'+KG*R*KG.';
end
end

function plotToGraph(name, posP, posV, deltaX_c, deltaV_c, mPEx, mPEv, varX, varV, time, carlos)
etime = repmat(time, carlos); %"extended" time. time array dup. for carlos

sgtitle(name);
subX = 1;
subY = 2;
name = "Posistion Error";
yLabel = "Difference (m)";
plotFilter(name, yLabel, posP, time, etime, deltaX_c, mPEx,varX,subX,subY,1);
name = "Velocity Error";
yLabel = "Difference (m/s)";
plotFilter(name, yLabel, posV,time,etime, deltaV_c, mPEv, varV, subX,subY,2);
end

function plotFilter(name, yl, P, time, etime, delta, mPE, sigma, subX, subY, pos)
subplot(subY, subX, pos);
title(name);
threeSigmaAboveP = 3*sqrt(P);
threeSigmaAboveData_pos = 3*sqrt(sigma) + mPE;
threeSigmaAboveData_neg = -3*sqrt(sigma) + mPE;
hold on
p4 = plot(etime, delta, '.', 'Color', 'b', 'DisplayName', 'Error from True');
p3 = plot(time, mPE, 'Color', 'r', 'LineWidth', 2, 'DisplayName', 'Mean Error');
p1 = plot(time, threeSigmaAboveP, 'LineWidth',2,  'Color','k', 'DisplayName', '3 Sigma P');
p2 = plot(time, -threeSigmaAboveP, 'LineWidth',2, 'Color','k');
p5 = plot(time, threeSigmaAboveData_pos, '--', 'LineWidth', 2, 'Color','b', 'DisplayName', '3 Sigma Data');
p6 = plot(time, threeSigmaAboveData_neg, '--', 'LineWidth', 2, 'Color','b');
ylabel(yl);
xlabel("Time (s)");
hold off

legend([p4(1), p3, p1, p5],'Orientation', 'Vertical');
end

function output = pull2x2byIndex(i)
loc = i*2 - 1;
output = loc:loc+1;
end