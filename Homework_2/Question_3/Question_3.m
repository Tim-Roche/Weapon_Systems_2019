%Timothy Roche
%Weapon Systems
%Question 3

value = 0:0.1:5;
data = [];
for i = value
    data = [data,CA_model(i)];
end

plot(value,data);
title("CA VS Mach");
xlabel("Mach");
ylabel("CA");

%------------CA_model.m----------------
function CA = CA_model(mach)
    if(mach < 0.3)
       CA = 0.5;
    elseif(mach > 1)
       CA = 0.3 + 0.7/mach;
    else
       CA = (0.5/0.7)*mach + 0.2857;
    end
end