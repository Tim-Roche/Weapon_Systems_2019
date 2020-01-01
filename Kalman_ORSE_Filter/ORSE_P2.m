%Timothy Roche
%Final Project Weapon Systems
%Part 2-2.1: ORSE-1D
clear all;
close all;

rng('default');
SIM_TIME = 50;
dt = 1;
TP = int64(SIM_TIME/dt) + 1;

%Initial Condition
inits = [0;    %Initial Posistion
         200]; %Initial Velocity

%Number of Monte Carlos
carlos = 500;

setAcc = 'DWNA'; %Truth acceleration is white noise

[posP, velP, deltaX_c, deltaV_c,  mPE, varA, time] = ORSE_1D_MC(setAcc,SIM_TIME,dt,inits,TP,carlos);
               
graphName = "1D ORSE Filter -- a = White Noise";
figure(1);
movegui('west');
plotToGraph(graphName, posP, velP, deltaX_c, deltaV_c,  mPE, varA, time);

setAcc = '15'; %Truth acceleration is 15m/s^2

[posPa, velPa, deltaX_ca, deltaV_ca, mPEa, varAa, time] =...
                  ORSE_1D_MC(setAcc,SIM_TIME,dt,inits,TP,carlos);
              
graphName_a = "1D ORSE Filter -- a = 15m/s^2";
figure(2);
movegui('east');
plotToGraph(graphName_a, posPa, velPa, deltaX_ca, deltaV_ca, mPEa, varAa, time);

%----------Functions-------------
function [posP, velP, deltaX, deltaV,  mPE, varA, time] = ORSE_1D_MC(setAcc,SIM_TIME,dt,inits,TP,carlos)
               % = ORSE_1D_MC(setAcc,SIM_TIME,dt,inits,TP,carlos)
    
    %ORSE Filter + 500 rounds of Monte Carlo
    deltaX = zeros(carlos, TP); 
    deltaV = zeros(carlos, TP);
    
    
    posP = zeros(1,TP);
    velP = zeros(1,TP);
    
    allState = zeros(carlos, TP, 2);
    Mup = zeros(carlos, TP, 2);
    
    %Begin Monte Carlos
    for c = 1:carlos
        [Xup, p, state, time] = ORSE_1D(setAcc,SIM_TIME,dt,inits,TP);
        Mup(c, :, 1) = Xup(1, :);
        Mup(c, :, 2) = Xup(2, :);
        for i = 1:TP
            %Getting Pos and Vel Variance
            p2x2 = p(:,specialIndex(i)); 
            posP(i) = p2x2(1,1);
            velP(i) = p2x2(2,2);
            
            allState(c, i, 1) = state(1, i);
            allState(c, i, 2) = state(2, i);
        end
        deltaX = Mup(:, :, 1) - allState(:, :, 1);
        deltaV = Mup(:, :, 2) - allState(:, :, 2);
    end
    %Calculating the mean at each time interval
    mPEx = mean(deltaX);
    mPEv = mean(deltaV);
    mPE = [mPEx; mPEv];
    
    %Calculating the varience at each time interval
    varX = var(deltaX);
    varV = var(deltaV);
    varA = [varX; varV];
end

function [x, S, state, time] = ORSE_1D(setAcc,SIM_TIME,dt,inits,TP)
%Setting acceleration whether that be white noise or a constant value
providedAcc = 0; 
DWNA = true;
if(setAcc ~= "DWNA") %If setAcc is DWNA then we want white noise
    DWNA = false;
    providedAcc = str2num(setAcc);
end

time = 0:dt:SIM_TIME;

%ORSE Related
lamda = 9;
G = [(1/2)*dt^2; dt];

%Defining Key Matrixes
x = zeros(2, TP);
S = zeros(2,TP*2);
D = [0;0]; 
M = zeros(2, TP*2);

%Matrixes
A = [1, dt; 0, 1]; %Transistion Matrix

%Conversion Matrix
H = [1,0];

%Measurement Errors
R_true = 900;
R_filt = 0.95*R_true; 

%Noise
W_true = 12;

M(:, specialIndex(1)) = [1, (1/dt); (1/dt), (2/dt)^2]*R_true; 
S(:, specialIndex(1)) = M(:, specialIndex(1));% D*lamda^2*D.';

state = zeros(2, TP);

%Getting the first truth reading	
previousState = inits;
initalSensorZ = inits(1) + normrnd(0, sqrt(R_true));

for i = 1:TP
    %The main iteration loop
    B = [(1/2)*dt^2; dt]; %Control Matrix for accel.
    
    %Find true posistion for this time interval
    acc = providedAcc; 
    if(DWNA) %Check to see if this is Part 1 or Part 1.1
        acc = normrnd(0, sqrt(W_true)); %Make Acc white noise instead
    end
    state(:,i) = A*previousState + B*acc;
    previousState = state(:,i); %Gets around indexing issue when i = 1
    
    %Get New Measured Value
    noiseX = normrnd(0, sqrt(R_true)); %Adding noise to the reading
    meas = state(1,i) + noiseX; 

    if(i == 1) 
        %Get an initial velocity and update our posistion 
        x(:, 1) = [meas; (meas-initalSensorZ)/dt];  
    
    %If this is not our first run
    else   
    %New Predicted State
    x_pC = A*x(:, i-1);
    M_mC = A*M(:, specialIndex(i-1))*A.';
    D = A*D + G;
    
    %Gain Computation
    S(:, specialIndex(i)) = M_mC + D*lamda^2*D.';
    KG = (S(:, specialIndex(i))*H.')/(H*S(:, specialIndex(i))*H.' + R_filt);
    
    %Calculate Current State
    x(:, i) = x_pC + KG*(meas - H*x_pC);
    M(:, specialIndex(i)) = (eye(2) - KG*H)*M_mC*(eye(2)-KG*H).'+KG*R_filt*KG.';
    D = (eye(2) - KG*H)*D;
    end
end
end

function plotToGraph(name, posP, posV, deltaX_c, deltaV_c, mPE, varA, time)
%This function formats the output graph
varX = varA(1, :);
varV = varA(2, :);
mPEx = mPE(1, :);
mPEv = mPE(2, :);
sgtitle(name);
subX = 1;
subY = 2 + 1; %Plus one for the legend
name = "Posistion Error";
yLabel = "Difference (m)";
plotFilter(name, yLabel, posP, time, deltaX_c, mPEx,varX,subX,subY,1);
name = "Velocity Error";
yLabel = "Difference (m/s)";
plotFilter(name, yLabel, posV,time, deltaV_c, mPEv, varV, subX,subY,2);
end

function output = specialIndex(i)
%For 2x2 or 6x6 matrixes a special indexing system in used defined as:
% Index:   1  ,  2  ,  3  ....		
%Actual: [2:2],[2;2],[2;2]....		

%size x size special matrix
size = 2;
loc = i*size - 1;
output = loc:loc+(size-1);
end