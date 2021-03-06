function outputdata = testBatteryModel(loads)
% testPrognoser   Test Prognoser class for Battery model
%
%   Copyright (c) 2016 United States Government as represented by the
%   Administrator of the National Aeronautics and Space Administration.
%   No copyright is claimed in the United States under Title 17, U.S.
%   Code. All Other Rights Reserved.
% close all

% Create battery model
addpath('C:\Users\mrmehta\Documents\GitHub\PrognosticsModelLibrary\MATLAB')
import Battery.*
battery = Battery.Create;
oneC = 2.1; %Default value
if nargin==1
    cratio= loads(1)./oneC;
    trueEOD = 3600./cratio;
    trueEODinHr = trueEOD./3600;
    loadval = oneC./trueEODinHr;
    fprintf('Maximum current: %g\n',loadval);
    battery.inputEqnHandle = @(P,t)Battery.InputEqn(P,t,loads);
    [Ttosim,~,~,Z] = battery.simulateToThreshold();
    trueEOD = Ttosim(end);
else
    errordlg('Not implemented yet. The load as the (ISC current, fault time) needs to be supplied with the function.')
end
parameters = Battery.Parameters;
outputdata.time = Ttosim;
outputdata.voltage = Z(2,:);
outputdata.EOD = trueEOD; 
outputdata.temperature= Z(1,:);
outputdata.ambTemp = parameters.x0.Tb;


