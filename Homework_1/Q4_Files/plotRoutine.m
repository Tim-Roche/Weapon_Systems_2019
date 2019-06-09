%Timothy Roche
%Question 4 HW 1
%Weapon Systems

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotRoutine.m
% Plot Function File
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotRoutine(x)
    X_SIZE = numel(x);
    y = zeros(1, X_SIZE);
    z = zeros(1, X_SIZE);
    w = zeros(1, X_SIZE);
    
    for i = 1:numel(x)
        y(i) = (x(i))^2;
        z(i) = (x(i)+1)^2;
        w(i) = exp(-0.5*x(i));
    end
    plot(x,y);
    title("Homework Q4 Plots");
    hold on
    plot(x,z);
    plot(x,w);
    hold off
end