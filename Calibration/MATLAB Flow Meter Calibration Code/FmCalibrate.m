function FmCalibrate
%% Initial Reset
% Clear everything
close all; clear;
% clear all
% reset all ports; otherwise might be unable to connect to port
instrreset;

% create our clean up object for interrupt
cleanupObj = onCleanup(@cleanMeUp);

% serialPortOpened = 0;
% data = [];

%% User Control
% Linear least square method is used in this code. 

% NAME THE TEST FIRST
% The code will read from the previous data, or establish a new file if no
% data present.
% MUST CHANGE NAME OR DELETE PREVIOUS FILE IF DIFFERENT NUMBER OF SENSORS REPORT DATA
fileName = 'LoadCellCalibration_LC5';

% NAME THE FOLDER YOU WANT THE TEST TO BE IN
folderName = 'POSTHOTFIRE';

% set up data monitoring frequency
pauseTime = 0.05;

% set up table to collect data
ValveOpenTime = 3; % seconds

% density of the fluid in g/ml or g/cm^3
density = 1; 

dataTypes = ["double","double","double"];
dataLabels = ["FM_pin_count_per_sec","IsOpen","ml/count"];
sz = [1,3];
% testDataTable = table('Size',sz,'VariableTypes',dataTypes,'VariableNames',dataLabels);


% set up serial object
serialPortName = '/dev/cu.SLAB_USBtoUART'
% serialPortName = 'COM6'; % on Windows would be COMx
%s = serialport(serialPortName,115200);

%% Automatic
s = serial(serialPortName,'BaudRate',115200);

% open serial port
serialPortOpened = 1;
try
fopen(s);
flushinput(s);
% fscanf(s);
catch 
    serialPortOpened = 0;
    availablePorts = convertStringsToChars(serialportlist("all"));
    [~,numOfPorts] = size(availablePorts);
    portNames = [];
    for n = 1:numOfPorts
        if n == 1
            portNames = availablePorts{n};
        else
            portNames = [portNames, ', ',availablePorts{n}];
        end
    end

    ME = MException('MyComponent:fopenFailed',['Failed to fopen the serial ' ...
        'port, check the serial port name. Available ports: ' portNames]);
    throw(ME)

end
% remember to fclose(s) in the command windows after ctrl+C exit the
% infinite while loop so that other programs can use the port

% for storing data sequentially in data1 and data2
i = 1;

% read data

% accounts for any time delay from reading
if serialPortOpened == 1
while(1)
    str = split(fscanf(s));


    % data1 receives PT1
    data1 = str2double(str{1})
    % data2 receives PT2
    data2 = str2double(str{2})

    data(i,1) = data1
    data(i,2) = data2
%     testDataTable(i,:) = {data1,data2};

    i = i+1;
end
end
    function cleanMeUp()
    if serialPortOpened == 1


            % saves data to file (or could save to workspace)
            fprintf('saving test data as %s.xls\n',fileName)

            prompt = "What is the total mass in grams? (Numbers only) \n";
            massReading = input(prompt);
            %         str2double(reading);
            %         while (isnumeric(reading) == false)
            %             prompt = "What is the pressure gage reading?"
            %             reading = input(prompt);
            %         end
            %         reading = str2double(reading);

            
            data_filtered = data(data(:, 2) == 1, :)
            FinalVolume = massReading/density;
            fmConstant = FinalVolume/sum(data_filtered(:,1))
            
            sizeD = size(data_filtered);
            constantColumn = [fmConstant;zeros(sizeD(2)-1,1)];

            testDataTable = array2table([data,constantColumn],"VariableNames",dataLabels);

            fileString = fileName + ".xls";

            if dataFileExist == 1
                writetable(testDataTable,fileString,'WriteMode','overwrite')
                movefile(fileString,folderName);
            else
                writetable(testDataTable,fileName,"FileType","spreadsheet");
                movefile(fileString,folderName);

            end
            dataProcessingGraphing(processArray,endsol)

    end
    
        fclose(s);
        instrreset;
    end



end


