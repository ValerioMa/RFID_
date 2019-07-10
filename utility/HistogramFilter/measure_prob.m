function prob = measure_prob(measurement, predicted_measurement)

%%%%%%%%%%%%%%   PARAMETERS  %%%%%%%%%%%%%%%%%%
meas_std = 0.05;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gaussian_exponent = 0;
for i=1:numel(measurement)
    gaussian_exponent = gaussian_exponent + gaussProbExponent(predicted_measurement(i), measurement(i),meas_std);    
end
prob = exp(gaussian_exponent);
end


function v = gaussProbExponent(value,mu,std)
v = -(value-mu)^2/(2*std^2);
end