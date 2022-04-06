clear all

fileName = "Test_Data_2022-04-05/2022-04-05_234516_fm_test1.xls"
T = readtable(fileName)
A = table2array(T)
C4 = T{:,4}
NotOnes = C4(C4 ~= 1)