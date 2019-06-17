%Timothy Roche
%Weapon Systems
%Test Program For Question 3
value = 0:0.1:5;
data = [];
for i = value
    data = [data,CA_model(i)];
end

plot(value,data);
title("CA VS Mach");
xlabel("Mach");
ylabel("CA");