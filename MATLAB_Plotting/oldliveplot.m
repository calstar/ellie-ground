% Clear everything
close all; clear all;
% reset all ports; otherwise might be unable to connect to port
% make sure to install
instrreset;

% set up data monitoring frequency
pauseTime = 0.05;
% set up plotting frequency
plotTime = 0.03;

% frequency
fetchFrequency = 1/pauseTime;

% observation time window
observationInterval = 5;

% time conversion factor
timeFactor = 24 * 60 * 60;

% set up serial object
% serialPortName = 'COM10'; % on Windows would be COMx
serialPortName = 'COM6'; % Hubert's Device
s = serial(serialPortName,'BaudRate',115200);

% open serial port
fopen(s);
% remember to fclose(s) in the command windows after ctrl+C exit the
% infinite while loop so that other programs can use the port

% set up plot
f = figure;
f.Position = [300 -100 800 650];
t = tiledlayout(6,1);

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

% for storing data sequentially in data1 and data2
i = 1;
% read data
flushinput(s);
fscanf(s)

% accounts for any time delay from reading
timeZeroer = 0;

while(1)
    startTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
    startTime=startTime(21:23);
    str = split(fscanf(s));
    % each data line represents one sensor data
    timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    if i == 1
        timeZeroer = str2double(str{1});
        timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    end

    % data1 receives flowrate in L/min
    % data1(i) = str2double(str{2})/10000;

    % data1 receives PT1
    data1(i) = str2double(str{2})*2.3013*10^(-5)+15.977;
    % data2 receives PT2
    data2(i) = str2double(str{3})*2.3013*10^(-5)+15.977;
    % data3 receives PT3
    data3(i) = str2double(str{4})*2.3013*10^(-5)+15.977;
    % data4 receives PT4
    data4(i) = str2double(str{5})*2.3013*10^(-5)+15.977;
    % data5 receives PT5
    data5(i) = str2double(str{6})*2.3013*10^(-5)+15.977;
    % data6 receives flowrate in L/min
    data6(i) = str2double(str{7})/10000;
    % data7 receives LC1
    % data7(i) = str2double(str{7});
    % data8 receives LC2
    % data8(i) = str2double(str{8});
    % data9 receives LC3
    % data9(i) = str2double(str{9});



    if (now() - timeControl) * 24 * 60 * 60 >= plotTime % plot every x seconds
        if timeInterval(end)-timeInterval(1) <= observationInterval


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
            plot(timeInterval,data5);
            title('Pressure Transducer 5')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax6);
            plot(timeInterval,data6);
            title('Flow Meter')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

        else % plot only the latest 5 seconds of data
            axes(ax1);
            plot(timeInterval(end-observationInterval/pauseTime:end),data1(end-observationInterval/pauseTime:end));
            % set the x limits so that only the last 5 seconds of data is
            % plotted
            title('Pressure Transducer 1')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax2);
            % change the number e.g "-6" according to trials.
            % Because data can only be sent at a rate, if the number is too big too much data will be subtracted--
            % the system is demanding to subtract more data than what came in in the first 5 seconds; 
            % if too small, then more than 5 seconds of data will be present at all time
            plot(timeInterval(end-observationInterval/pauseTime:end),data2(end-observationInterval/pauseTime:end));
            title('Pressure Transducer 2')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);

            axes(ax3);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-observationInterval/pauseTime:end),data3(end-observationInterval/pauseTime:end));
            title('Pressure Transducer 3')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);

            axes(ax4);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-observationInterval/pauseTime:end),data4(end-observationInterval/pauseTime:end));
            title('Pressure Transducer 4')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax5);
            % change the number e.g "-6" according to trials.
            plot(timeInterval(end-observationInterval/pauseTime:end),data5(end-observationInterval/pauseTime:end));
            title('Pressure Transducer 5')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

            axes(ax6);
            plot(timeInterval(end-observationInterval/pauseTime:end),data6(end-observationInterval/pauseTime:end));
            % set the x limits so that only the last 5 seconds of data is
            % plotted
            title('Flow Meter')

            xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
        end
        timeControl = now();
    end
    endTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
    endTime=endTime(21:23);

    timeDifference=str2num(endTime)-str2num(startTime);
    if timeDifference<pauseTime
        pause(timeDifference);
    end
    i = i+1;
end