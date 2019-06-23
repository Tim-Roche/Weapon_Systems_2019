%Timothy Roche
%Weapon Systems
%Question 3
function CA = CA_model_HW_2(mach)
    if(mach < 0.3)
       CA = 0.5;
    elseif(mach > 1)
       CA = 0.3 + 0.7/mach;
    else
       CA = (0.5/0.7)*mach + 0.2857;
    end
end