clear;
close all
fileName='HotfireData2';
table = readtable([fileName,'.xlsx']);
dataArray = table2array(table);
deletePreIndex = 1690; % There is a break in the data sheet, check index 1590 for Hotfire2Data
deletePostIndex = 250;
dataArray(1:deletePreIndex,:) = [];
dataArray(deletePostIndex:end,:) = [];
saveFileName = fileName+"_Processed";

dataTypes = ["double","double","double","double","double","double","double","double","double","double"];
dataLabels = ["time (s)","PT1 (psi)","PT2 (psi)","PT3 (psi)","PT4 (psi)","LC5 (lbs)","LC6 (lbs)","LC7 (lbs)","FM","S1","S2","commandedState","DAQState","Queue Size"];
ylabels = ["seconds","psi","psi","psi","psi","lb","lb","lb"];
% sz = [1,dataLength];

const = [0,0;
    1.1293e-04,12.4897;
    2.7716e-05,17.5329;
    1.1438e-04,16.2320;
    2.7673e-05,18.9369;
    -3.0272e-04,-0.1535;
    -3.4136e-04,0.8333;
    -2.4978e-04,1.6369];

% const = [0,0;
%     1.0789e-04,21.5850;
%     2.6509e-05,25.5954;
%     1.0949e-04,25.0202;
%     1.0603e-04,27.2010;
%     -3.0272e-04,-0.1535;
%     -3.4136e-04,0.8333;
%     -2.4978e-04,1.6369];

% const = [ones(8,1),zeros(8,1)];

maxPoints = [NaN,NaN];
minPoints = [NaN,NaN];
dataArray(:,1) = (dataArray(:,1)-dataArray(1,1))/1000;
for i = 2:length(const)
dataArray(:,i) = dataArray(:,i).*const(i,1)+const(i,2);
[minPoint,minIndex] = min(dataArray(:,i));
[maxPoint,maxIndex] = max(dataArray(:,i));
maxPoints = [maxPoints;maxPoint,maxIndex]; 
minPoints = [minPoints;minPoint,minIndex]; 
end

maxThrust = -sum(minPoints(6:8,1)-maxPoints(6:8,1))

table = array2table(dataArray,"VariableNames",dataLabels);
figure

% [~,col] = size(dataArray);
for k = 2:8
nexttile;
p = plot(dataArray(:,1),dataArray(:,k));
dtmax = datatip(p,dataArray(maxPoints(k,2),1),maxPoints(k,1));
if k >= 6
    dtmin = datatip(p,dataArray(minPoints(k,2),1),minPoints(k,1));
end
title(dataLabels(k));
xlabel(ylabels(1));
ylabel(ylabels(k));
% nexttile;
end


writetable(table,saveFileName,"FileType","spreadsheet");
% fileString = saveFileName + ".xls";
% movefile(fileString,folderName);



