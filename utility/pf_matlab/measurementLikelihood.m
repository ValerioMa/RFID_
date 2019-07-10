function [likelihood] = measurementLikelihood(pf, predictParticles, measurement, RFID) %#ok<INUSL>
%measurementLikelihood Compute the likelihood for each predicted
%   particles, which will later be used to compute importance weight. 

    % The measurement contains all state variables
    predictMeasurement  = rfidReadings(predictParticles',RFID)';
    

    % Calculate observed error between predicted and actual measurement
    % NOTE in this example, we don't have full state observation, but only
    % the measurement of current pose, therefore the measurementErrorNorm
    % is only based on the pose error.
    measurementError = predictMeasurement(:,:) -  measurement;
    
    % Normal-distributed noise of measurement
    % Assuming measure are uncorrelated with same covariance
    measurementStd = 0.05;
    
    % Convert error norms into likelihood measure. 
    gaussian_exp = zeros(size(measurementError,1),1);
    for k=1:size(measurementError,2)
        gaussian_exp = gaussian_exp - (measurementError(:,k)/measurementStd).^2;
    end
    likelihood = 1/(measurementStd*sqrt(2*pi))*exp(gaussian_exp); % no need to put the constant before gaussian
        
end
