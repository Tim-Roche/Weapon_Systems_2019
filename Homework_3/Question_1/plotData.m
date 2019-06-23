function plotData(xData, yData, name, xl, yl,subX, subY, POS,xPosMin, xPosMax)
    %Plotting Data
    subplot(subY,subX,POS);
    title(name);
    xlabel(xl);
    ylabel(yl);
    hold on
    temp = size(xData);
    ySize = temp(1); %So that this function works with different inputs
    for i = 1:ySize
        data_x = xData(i, :);
        data_y = yData(i, :);
        plot(data_x, data_y);
    end
    hold off
    xlim([xPosMin, xPosMax]);
end