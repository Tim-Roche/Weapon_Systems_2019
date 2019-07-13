%Timothy Roche
%Final Project Weapon Systems

SIM_TIME = 50;
dt = 2;
TOTAL_POINTS = int64(SIM_TIME/dt) + 1;

%Initial Condition
P_0 = 0; %Initial Posistion
V_0 = 200; %Initial Velocity

figure(1);
movegui('west');
trueAccel = NaN; %Truth acceleration is white noise
[x, p, truth, time] = kf(trueAccel, SIM_TIME, dt, P_0, V_0,TOTAL_POINTS);
graphName = "1D Kalman Filter -- a = White Noise";
plotToGraph(graphName, x, p, truth, time, TOTAL_POINTS);

figure(2);
movegui('east');
trueAccel = 15; %Truth acceleration is 15m/s^2
[x_a, p_a, truth_a, time_a] = kf(trueAccel, SIM_TIME, dt, P_0, V_0,TOTAL_POINTS);
graphName_a = "1D Kalman Filter -- a = 15m/s^2";
plotToGraph(graphName_a, x_a, p_a, truth_a, time_a, TOTAL_POINTS);

%----------Functions-------------
function [x, p, truth, time] = kf(acc, SIM_TIME, dt, P_0, V_0, TOTAL_POINTS)
time = 0:dt:SIM_TIME;

%Defining Key Matrixes
x = zeros(2, TOTAL_POINTS);
p = zeros(2,2,TOTAL_POINTS);

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
x(:,1) = [P_0;V_0]; %x_prediction
p(:,:,1) = [1, (1/dt); (1/dt), (2/dt)^2]*R_true; %Process Uncertanty

truth = zeros(2, TOTAL_POINTS);
truth(:, 1) = [P_0; V_0];

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
    p_pC = A*p(:, :, i-1)*A.' + Q; %Assuming no error
    
    %Gain Computation
    KG = (p_pC*H.')/(H*p_pC*H.' + R);
    
    %Calculate Current State
    x(:, i) = x_pC + KG*(z - H*x_pC);
    p(:, :, i) = (eye(2) - KG*H)*p_pC; %(eye(2) - KG*H)*p_pC*(eye(2)-KG*H).'+KG*R*KG.';
end
end

function plotToGraph(name, x, p, truth, time, TOTAL_POINTS)
%Plotting Problem 1
posP = zeros(1,TOTAL_POINTS);
velP = zeros(1,TOTAL_POINTS); 
deltaX = zeros(1, TOTAL_POINTS);
deltaV = zeros(1, TOTAL_POINTS);
mPEx = zeros(1, TOTAL_POINTS); %mean posistion error distance
mPEv = zeros(1, TOTAL_POINTS); %mean posistion error velocity
for i = 1:TOTAL_POINTS
    posP(i) = p(1,1,i);
    velP(i) = p(2,2,i);
    deltaX(i) = x(1,i) - truth(1, i);
    deltaV(i) = x(2,i) - truth(2, i);
    mPEx(i) = mean(deltaX(1:i));
    mPEv(i) = mean(deltaV(1:i));
end

posP = zeros(1,TOTAL_POINTS);
velP = zeros(1,TOTAL_POINTS); 
deltaX = zeros(1, TOTAL_POINTS);
deltaV = zeros(1, TOTAL_POINTS);
mPEx = zeros(1, TOTAL_POINTS); %mean posistion error distance
mPEv = zeros(1, TOTAL_POINTS); %mean posistion error velocity
for i = 1:TOTAL_POINTS
    posP(i) = p(1,1,i);
    velP(i) = p(2,2,i);
    deltaX(i) = x(1,i) - truth(1, i);
    deltaV(i) = x(2,i) - truth(2, i);
    mPEx(i) = mean(deltaX(1:i));
    mPEv(i) = mean(deltaV(1:i));
end

sgtitle(name);
subX = 1;
subY = 2;
plotFilter("Posistion Error", "Difference (m)", posP, time, deltaX,mPEx, subX,subY,1);
plotFilter("Velocity Error", "Difference (m/s)", velP, time, deltaV,mPEv, subX,subY,2);
end

function plotFilter(name, yl, P, time, delta, mPE, subX, subY, pos)
subplot(subY, subX, pos);
title(name);
threeSigmaAbove = 3*sqrt(P);
hold on
plot(time, threeSigmaAbove, 'LineWidth',2,  'Color','k');
plot(time, -threeSigmaAbove, 'LineWidth',2, 'Color','k');
plot(time, mPE, 'Color', 'r');
plot(time, delta, 'Color', 'b');
ylabel(yl);
xlabel("Time (s)");
hold off
end