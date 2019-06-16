%Timothy Roche
%Weapon Systems
function CA = CA_model(mach)
    if(mach < 0.3)
       CA = 0.5;
    elseif(mach > 1)
       CA = 0.3 + 0.7/mach;
    else
       CA = (0.5/0.7)*mach + 0.2857;
    end
end