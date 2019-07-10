function [predictMeasurement] = MeasurementFcn(x, RFID)
%MeasurementFcn Compute the measure from a point x.

    % The measurement contains all state variables
    predictMeasurement  = rfidReadings(x,RFID)';
end
