%Timothy Roche
%Weapon Systems
%For Question 4
%Makes sure the graphs dont go below the ground
%Function acts like a buffer (passes through the data) 
%unless if the datapoint is below 0
function buffered = bufferAboveZero(data)
    buffered = data;  
    if(data < 0)
        buffered = 0;
    end
end