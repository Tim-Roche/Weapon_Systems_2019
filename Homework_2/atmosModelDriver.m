%Timothy Roche
%Weapon Systems
%Test Program for AtmosDriver

test_height = 25002;
test_velocity = 0;

[t,p,density, mach, Q] = atmosModel(test_height, test_velocity);
disp("t")
disp(t);
disp("p");
disp(p);
disp("mach");
disp(mach);
disp("Q");
disp(Q);

data = []
x = 1:100:30000;
hold on;
for v = 1:500:3000
    for h = x
        [t,p, density, mach, Q] = atmosModel(h, 100);
        data = [data, t];
    end
    plot(x/1000,data);t
    data = [];
end
hold off;

