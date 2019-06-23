value = 0:0.1:5;
newModel = [];

for i = 0:50
    newModel = [newModel, CA_model(i*0.1)];
end

plotData(value, newModel, "New CA Model","Value", "Output", 2,1,1,-inf,inf);

