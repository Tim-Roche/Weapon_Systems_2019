%Timothy Roche
%Weapon Systems
%Question 2
%THIS IS THE UPDATED CA MODEL 

function CA = CA_model(mach)
    if(mach < 0.5)
       CA = 0.04;
    elseif(mach > 1)
       CA = 0.02 + 0.06/mach;
    else
       CA = (0.04/0.5)*mach;
    end
end