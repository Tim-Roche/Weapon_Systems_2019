%Timothy Roche
%Question 3 HW 1
%Weapon Systems

x = 0:0.1:20;
y = zeros(1,numel(x));
z = zeros(1,numel(x));
w = zeros(1,numel(x));

for i = 1:numel(x)
    y(i) = sin(x(i));
    z(i) = cos(x(i));
    w(i) = sqrt(x(i)^2 + (y(i)^2 + 1)); 
end

plot(x, y);
title("Homework 1 Question 3 Plots");
hold on
plot(x, z);
plot(x, w);
hold off

