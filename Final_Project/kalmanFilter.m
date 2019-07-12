%Timothy Roche
%Final Project Weapon Systems
%All Hail: http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190

SIM_TIME = 50;
dt = 2;
TOTAL_POINTS = int64(SIM_TIME/dt) + 1;

time = 0:dt:SIM_TIME;

%Defining Key Matrixes
x = zeros(2, TOTAL_POINTS);
p = zeros(2,2,TOTAL_POINTS);

%Initial Condition
P_0 = 0; %Initial Posistion
V_0 = 200; %Initial Velocity
x(:,1) = [P_0;V_0]; %x_prediction
p(:,:,1) = [1, 1; 1, 1]; %Process Uncertanty

%Measurement Errors
R_true = 900;
R = 0.95*R_true; 

%Matrixes
A = [1, dt; 0, 1]; %Transistion Matrix
C = [1,0;0,1];

%Various Errors
w = 0;
Q = 0;

%"Conversion" Matrix
H = [1,0];%;1,0];

%Noise
W_true = 12;
W = 0.9*W_true;

truth = zeros(2, TOTAL_POINTS);
truth(:, 1) = [0; 200];

for i = 2:TOTAL_POINTS
    B = [(1/2)*dt^2; dt];
    
    %Find true posistion
    noiseA = normrnd(0, sqrt(W_true));
    truth(:,i) = A*truth(:,i-1) + B*noiseA;
    
    %Get New Measured Value
    noiseX = normrnd(0, sqrt(R_true));
    measuredX = truth(1,i) + noiseX;%C*Y_m + z; 
    previousX = x(1, i-1);
    z = previousX;

    %Process Noise
    Q = [(1/4)*dt^4, (1/2)*dt^3; (1/2)*dt^3, dt^2]*W;
    
    %New Predicted State
    x_pC = A*x(:, i-1) + w; %Assuming no acceleration or error
    p_pC = A*p(:, :, i-1) + Q; %Assuming no error
    
    %Gain Computation
    KG = (p_pC*H.')/((H*p_pC*H.') + R);
    
    %Calculate Current State
    x(:, i) = x_pC + KG*(z - H*x_pC);
    p(:, :, i) = (eye(2) - KG*H)*p_pC*(eye(2)-KG*H).'+KG*R*KG.';
end

figure(1);
posP = zeros(1,TOTAL_POINTS); 
for i = 1:TOTAL_POINTS
    posP(i) = p(1,1,i);
end

threeSigmaAbove = 3*arrayfun(sqrt, posP);

