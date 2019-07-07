function [x_a, y_a, x_v, y_v, x_x, y_x, AoA, timeArray, timef, distancef, velf, accNBody, aDrag, iDrag, aUp] = PMwithNormalForce(cnalpha, gamma, Nc, mass, sRef, INC, totalPoints)
    timef = 0;
    distancef = 0;
    velf = 0;
    lock = false;
    %Iteration Setup

    AoA = zeros(1, totalPoints);
   
    
    timeArray = zeros(1, totalPoints);
    accNBody = zeros(1, totalPoints);
    aDrag = zeros(1, totalPoints);
    iDrag = zeros(1, totalPoints);
    aUp = zeros(1, totalPoints);
    x_v = zeros(1, totalPoints);
    y_v = zeros(1, totalPoints);
    x_a = zeros(1, totalPoints);
    y_a = zeros(1, totalPoints);
    x_x = zeros(1, totalPoints);
    y_x = zeros(1, totalPoints);

    %Setting Initial Conditions
    v_o = 600;
    x_v(1) = v_o*cosd(gamma);
    y_v(1) = v_o*sind(gamma);
    x_x(1) = 0;
    y_x(1) = 0;
    AoA(1) = 0;
    timeArray(1) = 0;
    deltaT = INC;
    for i = 2:totalPoints
        timeArray(i) = timeArray(i-1)+deltaT;
        
        theta = AoA(i-1) + gamma; %In degrees
        accNBody(i) = Nc/(cosd(theta) * mass);

        vMag = sqrt(x_v(i-1)^2+y_v(i-1)^2); %Path. Theorm

        [t,p, density, mach, Q] = atmosModel(y_x(i-1),vMag);

        %N = Nc/cosd(theta);
        N = 9.81*mass;
        Nc = N*cosd(theta);
        aUp(i) = Nc/mass;
        [t_x,p_x, density_x, mach_x, Q_x] = atmosModel(y_x(i-1),x_v(i-1));
        k_x = (1/2)*density_x*CA_model(mach_x)*sRef;
        
        [t_y,p_y, density_y, mach_y, Q_y] = atmosModel(y_x(i-1),y_v(i-1));
        k_y = (1/2)*density_y*CA_model(mach_y)*sRef;
        
        Ax = k_x*vMag*x_v(i-1);
        Ay = k_y*vMag*y_v(i-1);
        A = sqrt(Ax^2 + Ay^2);

        L = N*cosd(AoA(i-1)) - A*sind(AoA(i-1));
        D = A*cosd(AoA(i-1)) + N*sind(AoA(i-1));
       
        x_a(i-1) = (-D*cosd(gamma) - L*sind(gamma))/mass;
        y_a(i-1) = -9.81 - (D*sind(gamma))/mass - (L*cosd(gamma))/mass;
        x_v(i) = x_v(i-1) + x_a(i-1)*deltaT;
        y_v(i) = y_v(i-1) + y_a(i-1)*deltaT;
        x_x(i) = x_x(i-1) + x_v(i-1)*deltaT;
        y_x(i) = y_x(i-1) + y_v(i-1)*deltaT; 
        aDrag(i) = A;
        iDrag(i) = D;
        
        AoA(i) = (N/(cnalpha*Q*sRef))*(180/pi()); %AoA in Degrees
        if(AoA(i) > 30) %Alpha clamp
            AoA(i) = 30;
        end
        gamma = asind(y_v(i-1)/sqrt(y_v(i-1)^2 + x_v(i-1)^2));
        if((y_x(i) <= 0)&&(~lock)) %This gets out final time, final vel...
            lock = true; 
            %lock Prevents statement from running again once we have an answer
            timef = timeArray(i); %Final time for the run
            velf = sqrt((x_v(i))^2 + (y_v(i)^2)); %Final velocity for the run
            distancef = x_x(i); %Final distance for the run
        end
    end
end