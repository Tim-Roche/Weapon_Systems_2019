%Questions:
%Is S a 6x6 and H a 2x3? because if true than how KG? cannot do that math
%What Should we set M to be as default?
%Is it fair to assume that M=P as Y = 0? (Error due to target model error)
%If not how do I get that?
%How is this weighted so I know how badly I am going to fail this class?

%Timothy Roche
%Final Project Weapon Systems
rng('default');
SIM_TIME = 50;
dt = 2;
TOTAL_POINTS = int64(SIM_TIME/dt) + 1;

%Initial Condition
P_0 = [0; 0; 400000]; %Initial Posistion
V_0 = [0; 400; 0]; %Initial Velocity
inits = [P_0; V_0];

%Number of Monte Carlos
carlos = 500;

setAcc = '9.8'; %Truth acceleration is 15m/s^2
[posP_acc, velP_acc, deltaX_c_acc, deltaV_c_acc, mPEx_acc, mPEv_acc, varX_acc, varV_acc, time] = Kf_MC(setAcc,SIM_TIME,dt,inits,TOTAL_POINTS,carlos);
graphName_a = "3D ORSE Filter";
figure(1);
movegui('east');
plotToGraph(graphName_a, posP_acc, velP_acc, deltaX_c_acc, deltaV_c_acc, mPEx_acc, mPEv_acc, varX_acc, varV_acc, time, carlos);

function [posP, velP, deltaX_tran, deltaV_tran, mPEx, mPEv, varX, varV, time] = Kf_MC(setAcc,SIM_TIME,dt,inits,TOTAL_POINTS,carlos)
    %Kalman Filter + 500 rounds of Monte Carlo
    
    deltaX_c = zeros(carlos,TOTAL_POINTS,3);
    deltaV_c = zeros(carlos,TOTAL_POINTS,3);
    
    posP = zeros(3,TOTAL_POINTS);
    velP = zeros(3,TOTAL_POINTS); 
    
    for c = 1:carlos
        [x, M, truth, time] = kf(setAcc,SIM_TIME,dt,inits,TOTAL_POINTS);
        for i = 1:TOTAL_POINTS
            p6x6 = M(:, pull6x6byIndex(i));
            for d = 1:3
                deltaX_c(c, i, d)  = x(d, i) - truth(d, i);
                posP(d, i) = p6x6(d,d);
            end
            for d = 4:6
                deltaV_c(c, i, d-3) = x(d, i) - truth(d, i);
                velP(d-3, i) = p6x6(d,d);
            end
        end
    end
    
    deltaX_tran = zeros(3, TOTAL_POINTS*carlos);
    deltaV_tran = zeros(3, TOTAL_POINTS*carlos);
    for d = 1:3
        deltaDemX = deltaX_c(:,:,d);
        deltaDemV = deltaV_c(:,:,d);
        
        mPEx(d) = mean(deltaDemX);
        mPEv(d) = mean(deltaDemV);
        
        varX(d) = var(deltaDemX);
        varV(d) = var(DeltaDemV);
        
        deltaX_tran(d, :) = reshape(deltaDemX.', 1, []);
        deltaV_tran(d, :) = reshape(deltaDemY.', 1, []);
    end
end

%----------Functions-------------
function [x, M, truth, time] = kf(setAcc,SIM_TIME,dt,inits,TOTAL_POINTS)
providedAcc = str2num(setAcc);


time = 0:dt:SIM_TIME;

%Defining Key Matrixes
x = zeros(6, TOTAL_POINTS);

%Matrixes
A = eye(6,6) + [0,0,0,dt,0,0;0,0,0,0,dt,0;0,0,0,0,0,dt;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0]; %Transistion Matrix

%"Conversion" Matrix
H = [0,1,0;0,0,1];
%ORSE things
lamda = 9;
G = [(1/2)*dt^2;(1/2)*dt^2;(1/2)*dt^2; dt;dt;dt];

%Measurement Errors
R_true = 900;
R = 0.95*R_true; 
 
%Noise
W_true = 12;
W = 0.9*W_true;

M = zeros(6, TOTAL_POINTS*6);
ROW1 = [1,        (1/dt), (2/dt^2), (3/dt^3), (4/dt^4), (5/dt^5)];
ROW2 = [  (1/dt), (2/dt^2), (3/dt^3), (4/dt^4), (5/dt^5), (6/dt^6)];
ROW3 = [(2/dt^2), (3/dt^3), (4/dt^4), (5/dt^5), (6/dt^6), (7/dt^7)];
ROW4 = [(3/dt^3), (4/dt^4), (5/dt^5), (6/dt^6), (7/dt^7), (8/dt^8)];
ROW5 = [(4/dt^4), (5/dt^5), (6/dt^6), (7/dt^7), (8/dt^8), (9/dt^9)];
ROW6 = [(5/dt^5), (6/dt^6), (7/dt^7), (8/dt^8), (9/dt^9), (10/dt^10)];
M(:, pull6x6byIndex(1)) = [ROW1; ROW2; ROW3; ROW4; ROW5; ROW6 *R_true]; 

D = zeros(6,1);

truth = zeros(6, TOTAL_POINTS);

previousTruth = inits;
initalSensorZ = inits(1) + normrnd(0, sqrt(R_true), [3,1]);
for i = 1:TOTAL_POINTS
    B_dis = eye(3)*((1/2)*dt^2);
    B_vel = eye(3)*dt;
    B = [B_dis; B_vel]; %Control Matrix for accel.
    
    accVector = [0;0;-1];
    %Find true posistion
    acc = providedAcc*accVector; 
    
    truth(:,i) = A*previousTruth + B*acc;
    previousTruth = truth(:,i); %Gets around indexing issue when i = 1
    
    %Get New Measured Value
    noiseX = normrnd(0, sqrt(R_true), [3,1]);
    z = truth(1,i) + noiseX; %z = measured value not the z dem!!!

    if(i == 1)
        x(:, 1) = [z; (z-initalSensorZ)/dt];  
    else
        
    %New Predicted State
    x_pC = A*x(:, i-1);
    M_mC = A*M(:, pull6x6byIndex(i-1))*A.';
    D = A*D + G;
    
    %Gain Computation
    S = M_mC + D*lamda^2*D.';
    KG = (S*H.')/(H*S*H.' + R);
    
    %Calculate Current State
    x(:, i) = x_pC + KG*(z - H*x_pC);
    M(:, pull6x6byIndex(i)) = (eye(2) - KG*H)*M_mC*(eye(2)-KG*H).'+KG*R*KG.';
    D = (eye(2) - KG*H)*D;
    end
end
end

function plotToGraph(name, posP, posV, deltaX_tran, deltaV_tran, mPEx, mPEv, varX, varV, time, carlos)
etime = repmat(time, carlos); %"extended" time. time array dup. for carlos

subX = 3;
subY = 2;

dir = ["x-dir"; "y-dir"; "z-dir"];

for d = 1:3
sgtitle(name);
name = dir(d) + " Posistion Error";
yLabel = "Difference (m)";
plotFilter(name,yLabel,posP(d, :),time,etime,deltaX_tran(d), mPEx(d),varX(d),subX,subY,d);

name = dir(d) + " Velocity Error";
yLabel = "Difference (m/s)";
plotFilter(name,yLabel,posV(d, :),time,etime,deltaV_tran(d),mPEv(d),varV(d),subX,subY,d+3);
end
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

function output = pull6x6byIndex(i)
loc = i*6 - 1;
output = loc:loc+5;
end