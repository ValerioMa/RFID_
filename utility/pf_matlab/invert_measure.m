function [circles] = invert_measure(measurement,RFID)

% Define the possible solution
segni(1,:)     = [ -1, -1];
segni(end+1,:) = [  1,  1];
segni(end+1,:) = [ -1,  1];
segni(end+1,:) = [  1, -1];


% Find all the point with the same measure
circonference = zeros(6,0);  % [x;y;R;delta0] 

for i = 1:size(segni,1)
    sol = segni(i,:).*[ acos(measure(1)),  acos(measure(2))];
    
end
end