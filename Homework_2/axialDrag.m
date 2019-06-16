%Timothy Roche
%Weapon Systems
END = 5;
INC = 0.1
x = 0:INC:END;
len = numel(x);
y = zeros(1, len);
for i = 1:len
    disp(i);
    y(i) = axDrag(x(i));
end
plot(x,y);
title("CA VS Mach");
xlabel("Mach");
ylabel("CA");

function CA = axDrag(mach)
    if(mach < 0.3)
       CA = 0.5;
    elseif(mach > 1)
       CA = 0.3 + 0.7/mach;
    else
       CA = (0.5/0.7)*mach + 0.2857;
    end
end

