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
oneC = 2.021;
materialproperties = struct();
if nargin==1
    cratio= loads(1)./oneC;
    trueEOD = 3600./cratio;
    trueEODinHr = trueEOD./3600;
    loadval = oneC./trueEODinHr;
    fprintf('Maximum current: %g\n',loadval);
    battery.inputEqnHandle = @(P,t)Battery.InputEqn(P,t,loads);
    materialproperties.anodetype = 'SiC'; % Graphite, SiC
    materialproperties.cathodetype = 'NMC811'; % LCO, NMC811
    battery.P = Battery.Parameters(battery.P.qMobile,materialproperties);
    battery.P.modeltype = 'ISCSC';
    battery.P.VEOD = 1;

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
outputdata.VEOD = battery.P.VEOD;


