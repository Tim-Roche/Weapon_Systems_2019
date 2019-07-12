%Timothy Roche
%Final Project Weapon Systems
%All Hail: http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190

SIM_TIME = 30;
dt = 0.1;
TOTAL_POINTS = 300;

%Initial Condition
P_0 = 0; %Initial Posistion
V_0 = 200; %Initial Velocity
x_p(1) = [P_0;V_0]; %x_prediction
p_p(1) = [1; 1; 1; 1]; %Process Uncertanty

%Measurement Errors
R_true = 900;
R = 0.95*R_true; 

%Matrixes
A = [1, dt; 0, 1]; %Transistion Matrix
C = [1,0;0,1];

%Various Errors
z = [0;0]; %Error in the mechanism to make observ.
w = 0;
Q = 0;

%"Conversion" Matrix
H = [1,0;1,0];

%Noise
W_true = 12;
W = 0.9*W_true;

for i = 2:TOTAL_POINTS
    %Find true posistion
    x_true(:, i) = A*x_true(:,i-1);
    
    %Garbage-ify the true posistion for my sensor to measurement
    variance = var(Y_m);
    Y_m(:,1) = [1;1]; %PLACEHOLDER!!!
    
    %Process Noise
    Q = [(1/4)*dt^4, (1/2)*dt^3; (1/2)dt^3, dt^2]*W;
    
    %New Predicted State
    x_pC = A*x(:, i-1) + w; %Assuming no acceleration or error
    p_pC = A*p(:, :, i-1) + Q; %Assuming no error
    
    %Gain Computation
    KG = (p_pC*H.')/((H*p_pC*H.') + R);
    
    %Get New Measured Value
    Y = C*Y_m + z; 
    
    %Calculate Current State
    x(:, i) = x_pC + KG*(Y - H*x_pC);
    p(:, :, i) = (eye(2) - K*H)*p_pC*(I-K*H).'+K*R*K.';
end