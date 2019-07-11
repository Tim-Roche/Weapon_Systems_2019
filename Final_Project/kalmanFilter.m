%Timothy Roche
%Final Project Weapon Systems
%All Hail: http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190

SIM_TIME = 30;
delta = 0.1;
TOTAL_POINTS = 300;
TM = [1;2]

%Initial Condition
x_p(1) = [10;10]; %x_prediction
p_p(1) = [1; 1]; %Process Uncertanty

A = [1, delta; 0, 1]; %Transistion Matrix

H = [1,0;1,0];
R = 0; %Yet to be defined
C = [1,0;1,0];
Z = [0;0]; %Error in the mechanism to make observ.
for i = 2:TOTAL_POINTS
    %New Predicted State
    x_p(:,i) = A*x_p(:,i-1); %Assuming no acceleration or error
    p_p(:,i) = A*p_p(:,i-1); %Assuming no error
    
    %Gain Computation
    KG = (p_p(:,i)*H.')/((H*p_p(:,i)*H.') + R);
    
    %Get New Measured Value
    Y = C*Y_m + Z; 
    
    %Calculate Current State
    x = x_p(:,i) + KG*(Y - H*x_p(:,i));
    p = (eye(2) - K*H)*p_p(:,i);
end