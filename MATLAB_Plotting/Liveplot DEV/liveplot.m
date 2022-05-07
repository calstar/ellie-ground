function liveplot
% Clear everything
close all; clear all;
% reset all ports; otherwise might be unable to connect to port
instrreset;

% create our clean up object for interrupt
cleanupObj = onCleanup(@cleanMeUp);

% NAME THE TEST FIRST (only change the second part)
fileName = [datestr(now,'yyyy-mm-dd_HHMMSS'),'_test3'];

% set up data monitoring frequency
pauseTime = 0.05;
% set up plotting frequency
plotTime = 0.2;

% frequency
fetchFrequency = 1/pauseTime;

% observation time window
observationInterval = 5;
sizeCalculated = 0;
dataSize = 0;

% time conversion factor
timeFactor = 24 * 60 * 60;

% set up table to collect data
dataTypes = ["double","double","double","double","double","double","double", "double", "double", "double", "double", "double", "double"];
dataLabels = ["time","PT1","PT2","PT3","PT4","LC1","LC2", "LC3", "FM", "S1", "S2", "I", "S1S2"];
sz = [1,13];
testDataTable = table('Size',sz,'VariableTypes',dataTypes,'VariableNames',dataLabels);


% set up serial object
serialPortName = 'COM4'; % on Windows would be COMx
% serialPortName = '/dev/cu.SLAB_USBtoUART'
s = serial(serialPortName,'BaudRate',115200);

% open serial port
fopen(s);
% remember to fclose(s) in the command windows after ctrl+C exit the
% infinite while loop so that other programs can use the port

% set up plot
f = figure;
f.Position = [300 -100 800 650];
t = tiledlayout(12,1);

% First tile
ax1 = nexttile;
ax1.XColor = [1 0 0];
ax1.YColor = [1 0 0];


% Second tile
ax2 = nexttile;
ax2.XColor = [1 0 0];
ax2.YColor = [1 0 0];


% Third tile
ax3 = nexttile;
ax3.XColor = [1 0 0];
ax3.YColor = [1 0 0];

% Fourth Tile
ax4 = nexttile;
ax4.XColor = [1 0 0];
ax4.YColor = [1 0 0];

% Fifth Tile
ax5 = nexttile;
ax5.XColor = [1 0 0];
ax5.YColor = [1 0 0];

% Sixth Tile
ax6 = nexttile;
ax6.XColor = [1 0 0];
ax6.YColor = [1 0 0];

% Seventh Tile
ax7 = nexttile;
ax7.XColor = [1 0 0];
ax7.YColor = [1 0 0];

% Eigth Tile
ax8 = nexttile;
ax8.XColor = [1 0 0];
ax8.YColor = [1 0 0];

% Ninth Tile
ax9 = nexttile;
ax9.XColor = [1 0 0];
ax9.YColor = [1 0 0];

%Tenth Tile
ax10 = nexttile;
ax10.XColor = [1 0 0];
ax10.YColor = [1 0 0];

%Eleventh Tile
ax11 = nexttile;
ax11.XColor = [1 0 0];
ax11.YColor = [1 0 0];

%Twelfth Tile
ax12 = nexttile;
ax12.XColor = [1 0 0];
ax12.YColor = [1 0 0];

timeControl = now();

% for storing data sequentially in data1 and data2
i = 1;
% read data
flushinput(s);
fscanf(s);

% accounts for any time delay from reading
timeZeroer = 0;

strlengthWarned = false;

while (strlength(split(fscanf(s))) < 1)
    if strlengthWarned == false
        fprintf("No data/not enough data received through the serial port\n")
    end
    strlengthWarned = true;
    
end

while(1)
    startTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
    startTime=startTime(21:23);
    fscanf(s)
    str = split(fscanf(s)," ")
    % each data line represents one sensor data
    timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    if i == 1
        timeZeroer = str2double(str{1});
        timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    end


    % data1 receives flowrate in L/min
    % data1(i) = str2double(str{2})/10000;

    % data1 receives PT1
    data1(i) = str2double(str{2})*1.051*10^(-4)+26.55;
    % data2 receives PT2
    data2(i) = str2double(str{3})*2.6083*10^(-5)+22.4204;
    % data3 receives PT3
    data3(i) = str2double(str{4})*1.0449*10^(-4)+24.571;
    % data4 receives PT4
    data4(i) = str2double(str{5})*8.6532*10^(-5)+24.0528;
    % data5 receives LC1
    data5(i) = str2double(str{6});
    % data6 receives LC2
    data6(i) = str2double(str{7});
    % data7 receives LC3
    data7(i) = str2double(str{8});
    
    % data8 receives flowrate in samples/50ms, multiplies by calibration
    % coefficent [g/cycle] and 20 to convert samples per 50 ms to samples per second
    % (20 observation intervals of 50ms in 1 s)
    data8(i) = str2double(str{9})*.0763*20;

    data9(i) = str2double(str(9));
    
    data10(i) = str2double(str(10));
    
    data11(i) = str2double(str(11));
    
    data12(i) = str2double(str(12));
    
    testDataTable(i, :) = {timeInterval(i),data1(i),data2(i),data3(i),data4(i),data5(i),data6(i), data7(i), data8(i), data9(i), data10(i), data11(i), data12(i)};



    % data7 receives LC1
    % data7(i) = str2double(str{7});
    % data8 receives LC2
    % data8(i) = str2double(str{8});
    % data9 receives LC3
    % data9(i) = str2double(str{9});



% START OF PLOTTING

%     if (now() - timeControl) * 24 * 60 * 60 >= plotTime % plot every x seconds
%         if timeInterval(end)-timeInterval(1) <= observationInterval
% 
% 
%             axes(ax1);
%             plot(timeInterval,data1);
%             title('Pressure Transducer 1')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax2);
%             plot(timeInterval,data2);
%             title('Pressure Transducer 2')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             % ylim([-500000 100000]);
% 
%             axes(ax3);
%             plot(timeInterval,data3);
%             title('Pressure Transducer 3')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax4);
%             plot(timeInterval,data4);
%             title('Pressure Transducer 4')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax5);
%             plot(timeInterval,data5);
%             title('Load Cell 1')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax6);
%             plot(timeInterval,data6);
%             title('Load Cell 2')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             
%             axes(ax7);
%             plot(timeInterval,data7);
%             title('Load Cell 3')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             
%             axes(ax8);
%             plot(timeInterval,data8);
%             title('Flowmeter')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             
% 
% 
%         else % plot only the latest 5 seconds of data
%             if sizeCalculated == 0
%                 [~,dataSize] = size(data1);
%                 sizeCalculated = 1;
%             end
%         
%             dataSize
%             axes(ax1);
%          %   plot(timeInterval(end-observationInterval/pauseTime:end),data1(end-observationInterval/pauseTime:end));
%             plot(timeInterval(end-dataSize+1:end),data1(end-dataSize+1:end));
% 
%             % set the x limits so that only the last 5 seconds of data is
%             % plotted
%             title('Pressure Transducer 1')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax2);
%             % change the number e.g "-6" according to trials.
%             plot(timeInterval(end-dataSize+1:end),data2(end-dataSize+1:end));
%             title('Pressure Transducer 2')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             % ylim([-500000 100000]);
% 
%             axes(ax3);
%             % change the number e.g "-6" according to trials.
%              plot(timeInterval(end-dataSize+1:end),data3(end-dataSize+1:end));
%             title('Pressure Transducer 3')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             % ylim([-500000 100000]);
% 
%             axes(ax4);
%             % change the number e.g "-6" according to trials.
%              plot(timeInterval(end-dataSize+1:end),data4(end-dataSize+1:end));
%             title('Pressure Transducer 4')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax5);
%             % change the number e.g "-6" according to trials.
%              plot(timeInterval(end-dataSize+1:end),data5(end-dataSize+1:end));
%             title('Load Cell 1')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
% 
%             axes(ax6);
%              plot(timeInterval(end-dataSize+1:end),data6(end-dataSize+1:end));
%             % set the x limits so that only the last 5 seconds of data is
%             % plotted
%             title('Load Cell 2')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             
%             axes(ax7);
%              plot(timeInterval(end-dataSize+1:end),data7(end-dataSize+1:end));
%             % set the x limits so that only the last 5 seconds of data is
%             % plotted
%             title('Load Cell 3')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             
%             axes(ax8);
%              plot(timeInterval(end-dataSize+1:end),data8(end-dataSize+1:end));
%             % set the x limits so that only the last 5 seconds of data is
%             % plotted
%             title('Flowmeter')
% 
%             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%         end
%         timeControl = now();
%     end
%
% END OF PLOTTING


    endTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
    endTime=endTime(21:23);

    timeDifference=str2double(endTime)-str2double(startTime);
    if timeDifference<pauseTime
        pause(timeDifference);
    end
    i = i+1;
end
    function cleanMeUp()
        % saves data to file (or could save to workspace)
        fprintf('saving test data as %s.xls\n',fileName);
        setUpTest(['Test_Data_',datestr(now,'yyyy-mm-dd')],fileName,testDataTable);
        %         writetable(testDataTable,fileName,"FileType","spreadsheet");
        fclose(s);
        instrreset;
    end



end

function setUpTest(folderName,fileName,testDataTable)
if ~exist(folderName, 'dir')
    mkdir(folderName);
    fprintf("test data folder created\n");
else
    fprintf("folder already exists\n")
end
writetable(testDataTable,fileName,"FileType","spreadsheet");
fileString = fileName + ".xls";
movefile(fileString,folderName);

end



