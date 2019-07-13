function output = get2by2(A, i)
loc = i*2 - 1;
output = A(:, loc:loc+1);
end