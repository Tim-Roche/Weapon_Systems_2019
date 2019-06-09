%Timothy Roche
%Question 2 HW 1
%Weapon Systems

INC = 0.1;
MAX_VALUE = int64(floor(20/INC)); %Getting around floating point issues
i = 0;
x = [];
y = [];
while (i <= MAX_VALUE)
    newX = i*INC;
    x = [x, newX];
    y = [y, cos(newX)];
    i = i + 1;
end
plot(x,y,'o');
title("Homework 1 Question 2 Plots");