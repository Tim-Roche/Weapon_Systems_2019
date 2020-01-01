%Timothy Roche
%Final Project Weapon Systems
%Part 3: ORSE-3D
clear all;
close all;

rng('default');
SIM_TIME = 50;
dt = 0.5;
TP = int64(SIM_TIME/dt) + 1;

%Initial Condition
inits = [0;0;400000; %Posistion
         0;400;0];   %Velocity

%Number of Monte Carlos
carlos = 500;

setAcc = '9.81'; %Truth acceleration; Is made negative inside function
[posP_a, velP_a, deltaX_c_a, deltaV_c_a, mPEa, varAa, time] = ...
                            ORSE_3D_MC(setAcc,SIM_TIME,dt,inits,TP,carlos);
                        
graphName_a = "3D ORSE Filter";
figure(1);

plotToGraph(graphName_a, posP_a, velP_a, deltaX_c_a, deltaV_c_a, mPEa, varAa, time);

%----------Functions-------------
function [posP, velP, deltaX, deltaV, mPE, varA, time] = ORSE_3D_MC(setAcc,SIM_TIME,dt,inits,TP,carlos)
    %Kalman Filter + 500 rounds of Monte Carlo
   
    deltaX = zeros(carlos,TP,3);
    deltaV = zeros(carlos,TP,3);
    
    posP = zeros(3,TP);
    velP = zeros(3,TP); 
    
    %Begin Monte Carlos
    
    for c = 1:carlos
        [Xup, S, state, time] = ORSE_3D(setAcc,SIM_TIME,dt,inits,TP);
        for i = 1:TP
            %Getting Pos and Vel Varience
            p6x6 = S(:, specialIndex(i));
            for d = 1:3 %One for each demension
                deltaX(c, i, d)  = Xup(d, i) - state(d, i);
                posP(d, i) = p6x6(d,d);
            end
            for d = 4:6 %One for each demension
                deltaV(c, i, d-3) = Xup(d, i) - state(d, i);
                velP(d-3, i) = p6x6(d,d);
            end
        end
    end
    
    mPEx = zeros(3, TP);
    mPEv = zeros(3, TP);
    varX = zeros(3, TP);
    varV = zeros(3, TP);
    
    for d = 1:3 %One for each demension
        %Creating a varience matrix with only one demension for convenience
        deltaDemX = deltaX(:,:,d);
        deltaDemV = deltaV(:,:,d);
        
        %Calculating the mean at each time interval
        mPEx(d, :) = mean(deltaDemX);
        mPEv(d, :) = mean(deltaDemV);
        
        %Calculating the varience at each time interval
        varX(d, :) = var(deltaDemX);
        varV(d, :) = var(deltaDemV);
    end
    mPE = zeros(3, TP, 2);
    varA = zeros(3, TP, 2);
    mPE(:, :, 1) = mPEx;
    mPE(:, :, 2) = mPEv;
    varA(:, :, 1) = varX;
    varA(:, :, 2) = varV;
end

function [Xup, S, state, time] = ORSE_3D(setAcc,SIM_TIME,dt,inits,TP)
%Getting inputted acceleration
providedAcc = str2num(setAcc);

time = 0:dt:SIM_TIME;

%ORSE related
lamda = 9;

%Debate between two diffent G values can be found in report
%G = [(1/2)*dt^2; (1/2)*dt^2; (1/2)*dt^2; dt; dt; dt]; 
G = [0; 0; (1/2)*dt^2; 0; 0; dt];%A better model for G since no acc in x,y

%Defining Key Matrixes
Xup = zeros(6, TP);
M = zeros(6, TP*6);
S = zeros(6, TP*6);
D = zeros(6,1);

%Matrixes
A=eye(6,6)+[0,0,0,dt,0,0;
            0,0,0,0,dt,0;
            0,0,0,0,0,dt;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0]; %Transistion Matrix

%Conversion Matrix
H = [1,0,0,0,0,0;
     0,1,0,0,0,0;
     0,0,1,0,0,0]; %This H picks out only the posistion states

%Measurement Errors
R_true = 900;
R_filt = 0.95*R_true; 

state = zeros(6, TP);

%Initilization
M(:, specialIndex(1)) = eye(6)*R_filt;
S(:, specialIndex(1)) = M(:, specialIndex(1)); %Since D=0, M = S at t=0
accVector = [0;0;-1]; %Defining the direction of acceleration

initialSensorZ = zeros(3,1);

%Getting the first truth reading
previousState = inits;
for i = 1:3
    initialSensorZ(i) = inits(i) + normrnd(0, sqrt(R_true));
end

for i = 1:TP
    %The main iteration loop
    B_dis = eye(3)*((1/2)*dt^2);
    B_vel = eye(3)*dt;
    B = [B_dis; B_vel]; %Control Matrix for accel.
   
    %Find true posistion for this time interval
    acc = providedAcc*accVector; 
    state(:,i) = A*previousState + B*acc;
    previousState = state(:,i); %Gets around indexing issue when i = 1
    
    %Get New Measured Value
    noiseX = normrnd(0, sqrt(R_true), [3,1]);
    z = state(1:3,i) + noiseX; %z = measured value not the z dem!!!

    if(i == 1)
        %Get an initial velocity and update our posistion
        Xup(:, 1) = [z; (z-initialSensorZ)/dt];  
    %If this is not our first run
    else
    %New Predicted State
    x_pC = A*Xup(:, i-1);
    M_mC = A*M(:, specialIndex(i-1))*A.';
    D = A*D + G;
    
    %Gain Computation
    S(:, specialIndex(i)) = M_mC + D*lamda^2*D.';
    KG = (S(:, specialIndex(i))*H.')/(H*S(:, specialIndex(i))*H.' + R_filt);
    
    %Calculate Current State
    Xup(:, i) = x_pC + KG*(z - H*x_pC);
    M(:, specialIndex(i)) = (eye(6) - KG*H)*M_mC*(eye(6)-KG*H).'+KG*R_filt*KG.';
    D = (eye(6) - KG*H)*D;
    end
end
end

function plotToGraph(name, posP, posV, deltaX_3d, deltaV_3d, mPE, varA, time)
%This function formats the output graph
mPEx = mPE(:, :, 1); 
mPEv = mPE(:, :, 2);
varX = varA(:, :, 1);
varV = varA(:, :, 2);

sgtitle(name);
subX = 3;
subY = 2 + 1; %+ 1 for the legend

dir = ["X"; "Y"; "Z"]; %Names to be prepended on titles of graphs

for d = 1:3 %For each demension...
%Graph Posistion
name = dir(d) + " Posistion Error";
yLabel = "Difference (m)";
plotFilter(name,yLabel,posP(d, :),time,deltaX_3d(:,:,d), mPEx(d, :),varX(d, :),subX,subY,d);

%Graph Velocity
name = dir(d) + " Velocity Error";
yLabel = "Difference (m/s)";
plotFilter(name,yLabel,posV(d, :),time,deltaV_3d(:,:,d),mPEv(d, :),varV(d, :),subX,subY,d+3);
end
end 

function output = specialIndex(i)
%For 2x2 or 6x6 matrixes a special indexing system in used defined as
% Index:   1  ,  2  ,  3  ....	
%Actual: [2:2],[2;2],[2;2]....		

%size x size special matrix
size = 6;
loc = i*size - 1;
output = loc:loc+(size-1);
end