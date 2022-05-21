function liveplot
% Clear everything
close all; clear all;
% reset all ports; otherwise might be unable to connect to port
% instrreset;

% create our clean up object for interrupt
cleanupObj = onCleanup(@cleanMeUp);

% NAME THE TEST FIRST (only change the second part)
fileName = [datestr(now,'yyyy-mm-dd_HHMMSS'),'_test3'];

% set up data monitoring frequency
pauseTime = 0.05;
% set up plotting frequency
plotTime = 0.2;
% check variable size
SerialPrintSize = 14;

% frequency
% fetchFrequency = 1/pauseTime;

% observation time window
observationInterval = 5;

observationIntervalMillis = observationInterval*1000;

% time conversion factor
% timeFactor = 24 * 60 * 60;

% set up table to collect data
dataTypes = ["double","double","double","double","double","double","double","double","double"];
dataLabels = ["time","PT1","PT2","PT3","PT4","LC5","LC6","LC7","FM"];
sz = [1,9];
testDataTable = table('Size',sz,'VariableTypes',dataTypes,'VariableNames',dataLabels);


% set up serial object
% serialPortName = 'COM5'; % on Windows would be COMx
serialPortName = '/dev/cu.SLAB_USBtoUART'; % Please don't delete this line--Hubert uses it for testing
s = serial(serialPortName,'BaudRate',115200);
% s = serialport(serialPortName,115200);
% open serial port
fopen(s);
% remember to fclose(s) in the command windows after ctrl+C exit the
% infinite while loop so that other programs can use the port

% set up plot
f = figure;
f.Position = [300 -100 800 650];
% t = tiledlayout(6,1);

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


timeControl = now();
% timeControlContinuous = now();
idx = 0;
idxSet = 0;

% for storing data sequentially in data1 and data2
i = 1;
% read data
flushinput(s);
% if there is not reading coming from the serial port, this command will
% result in a timeout before proceeding (default 10s, but the following
% command changes the time)
set(s, 'TimeOut', 2)
% fscanf(s);


% accounts for any time delay from reading
timeZeroer = 0;

strlengthWarned = false;


while (strlength(split(fscanf(s))) < 1)
    if strlengthWarned == false
        fprintf("No data/not enough data received through the serial port, check board\n")
    end
    strlengthWarned = true;

end

fprintf("Receiving data now\n")
errorLength = 0;
correctLength = 0;

while(1)
%     startTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
%     startTime=startTime(21:23);
    rawStr = fscanf(s);
    linestr = split(rawStr,'←↵');
    str = split(linestr{1}," ");
    if length(str) ~= SerialPrintSize

        errorLength = errorLength + 1

        str
        rawStr
        length(str)
       
        
        continue;
    else 
        correctLength = correctLength + 1;
    end

    % each data line represents one sensor data
    timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    if i == 1
        timeZeroer = str2double(str{1});
        timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    end


   
    % data1 receives flowrate in L/min
    % data1(i) = str2double(str{2})/10000;

    % data1 receives PT1
    data1(i) = str2double(str{2})*1.0789e-04+21.5850;
    % data2 receives PT2
    data2(i) = str2double(str{3})*2.6509e-05+25.5954;
    % data3 receives PT3
    data3(i) = str2double(str{4})*1.0949e-04+25.0202;
    % data4 receives PT4
    data4(i) = str2double(str{5})*1.0603e-04+27.2010;
    % data5 receives PT5
    data5(i) = str2double(str{6})*-0.00030042+-0.144152513;
    % data6 receives flowrate in samples/50ms, multiplies by calibration
    % coefficent [g/cycle] and 20 to convert samples per 50 ms to samples per second
    % (20 observation intervals of 50ms in 1 s)
    data6(i) = str2double(str{7})*-0.000364984+0.445296444;
    data7(i) = str2double(str{8})*-0.00038533+-0.107909483;
    data8(i) = str2double(str{9});
%     data15 = str2double(str{14})


    testDataTable(i,:) = {timeInterval(i),data1(i),data2(i),data3(i),data4(i),data5(i),data6(i),data7(i),data8(i)};





% 
    if (now() - timeControl) * 24 * 60 * 60 >= plotTime % plot every x seconds
        if timeInterval(end)-timeInterval(1) < observationInterval


            axes(ax1);
            plot(timeInterval,data1);
            title('Pressure Transducer 1')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax2);
            plot(timeInterval,data2);
            title('Pressure Transducer 2')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);

            axes(ax3);
            plot(timeInterval,data3);
            title('Pressure Transducer 3')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax4);
            plot(timeInterval,data4);
            title('Pressure Transducer 4')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax5);
            plot(timeInterval,data5+data6+data7);
            title('Load Cell Combined')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax6);
            plot(timeInterval,data8);
            title('FM');

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

        else % plot only the latest 5 seconds of data
%             timeInterval(i-1)-timeInterval(1) < observationInterval
%             timeInterval(i)
%             timeInterval(i)-timeInterval(1) >= observationInterval
            if idxSet == 0
             idx =  floor(length(timeInterval))-1;
             idxSet = 1;

%                 idx
%             idxDumpTime = 1/(floor(length(timeInterval)/observationalInterval))
            end
           

%             if (now() - timeControlContinuous) * 24 * 60 * 60 >= idxDumpTime
%             idx

            axes(ax1);
            plot(timeInterval(end-idx:end),data1(end-idx:end));
            % set the x limits so that only the last 5 seconds of data is
            % plotted
            title('Pressure Transducer 1')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax2);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-idx:end),data2(end-idx:end));
            title('Pressure Transducer 2')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);

            axes(ax3);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-idx:end),data3(end-idx:end));
            title('Pressure Transducer 3')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);

            axes(ax4);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-idx:end),data4(end-idx:end));
            title('Pressure Transducer 4')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);




            axes(ax5);
            LoadCellDataLastCombined = data5+data6+data7;
            plot(timeInterval(end-idx:end),LoadCellDataLastCombined(end-idx:end));
            % set the x limits so that only last 5 seconds of data is
            % plotted
            title('Load Cell Total')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax6);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-idx:end),data8(end-idx:end));
            title('FM')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);


            timeControlContinuous = now();
%             end
        end
        timeControl = now();
    end
%     endTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
%     endTime=endTime(21:23);

%     timeDifference=str2double(endTime)-str2double(startTime);
%     if timeDifference<pauseTime
%         pause(timeDifference);
%     end
    i = i+1;
end
    function cleanMeUp()
        % saves data to file (or could save to workspace)
        fprintf('Saving test data as %s.xls\n',fileName);
        setUpTest(['Test_Data_',datestr(now,'yyyy-mm-dd')],fileName,testDataTable);
        %         writetable(testDataTable,fileName,"FileType","spreadsheet");
%         fclose(s);
        ratio = errorLength/correctLength
        clear s
        instrreset;
    end



end

function setUpTest(folderName,fileName,testDataTable)
if ~exist(folderName, 'dir')
    mkdir(folderName);
    fprintf("Test data folder created\n");
else
    fprintf("Folder already exists\n")
end
writetable(testDataTable,fileName,"FileType","spreadsheet");
fileString = fileName + ".xls";
movefile(fileString,folderName);

end
