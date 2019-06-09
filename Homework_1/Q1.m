%Timothy Roche
%Question 1 HW 1
%Weapon Systems

x = 0:0.1:20; 
y = zeros(1,numel(x)); %Makes y same length as x
for i = 1:numel(x) 
    y(i) = sin(x(i));
end
plot(x,y);
title("Homework 1 Question 1 Plot");