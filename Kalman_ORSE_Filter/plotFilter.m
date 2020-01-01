function plotFilter(name, yl, P, time, delta, mPE, sigma, subX, subY, pos)
%This function actually generates the plots
subplot(subY, subX, pos);
title(name);
threeSigmaAboveP = 3*sqrt(P);
threeSigmaAboveData_pos = 3*sqrt(sigma) + mPE;
threeSigmaAboveData_neg = -3*sqrt(sigma) + mPE;
hold on

p1 = plot(time, delta, '-', 'Color', 'b', 'DisplayName', 'Posistion Errors');
p2 = plot(time, mPE, 'Color', 'w', 'LineWidth', 2, 'DisplayName', 'Mean Error');
p3 = plot(time, threeSigmaAboveP, '--', 'LineWidth',2,  'Color','r', 'DisplayName', '3 Sigma of Errors (Theory)');
p4 = plot(time, -threeSigmaAboveP, '--', 'LineWidth',2, 'Color','r');
p5 = plot(time, threeSigmaAboveData_pos, '-', 'LineWidth', 2, 'Color','r', 'DisplayName', '3 Sigma of Errors (Data)');
p6 = plot(time, threeSigmaAboveData_neg, '-', 'LineWidth', 2, 'Color','r');
ylabel(yl);
xlabel("Time (s)");
hold off

%Create a legend by making a dummy graph 
posOfDummy = subY*subX; %Finding posistion
if(subX ~= 1)
    posOfDummy = posOfDummy-(subX-round(subX/2)); %Fix loc if xPos is not 1
end
dummyPlot = subplot(subY, subX, posOfDummy);
set(dummyPlot,'visible','off');
cords = get(dummyPlot,'Position');
leg = legend([p1(1), p2, p3, p5],'Orientation', 'Vertical');
set(leg,'Position',cords);

end