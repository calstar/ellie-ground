clear all

fileName = "Test_Data_2022-04-06/2022-04-06_003319_fm_test1.xls"
t = readtable(fileName)
A = table2array(t)
C1 = t{70:166,1}

timeInterval = 20836-20025
numTics = numel(C1)-8
cyclesPerSecond = (numTics/timeInterval)*1000
tt
pInitial = 146;
pFinal = 166;
pDiff = (pFinal-pInitial)*6894.76*1000
v = 4391e-6
R = 296.8
T = 283.1
flowRate = (pDiff*v)/(R*T*timeInterval/1000)

coefficient = flowRate/cyclesPerSecond
