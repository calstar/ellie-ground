clear all

fileName = "Test_Data_2022-04-06/2022-04-06_002444_fm_test1.xls"
t = readtable(fileName)
A = table2array(t)
C1 = 11358-10735

timeInterval = C1
numTics = 77
cyclesPerSecond = (numTics/timeInterval)*1000

pInitial = 92.9807;
pFinal = 106.9337;
pDiff = (pFinal-pInitial)*6894.76*1000
v = 4391e-6
R = 296.8
T = 283.1
flowRate = (pDiff*v)/(R*T*timeInterval/1000)

coefficient = flowRate/cyclesPerSecond
