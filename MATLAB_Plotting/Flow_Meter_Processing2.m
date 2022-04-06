
close all
figure
fm1=table2array(test1)-100

fm1=nonzeros(fm1)

fm1=fm1(fm1>=0)

df1=diff(fm1+1)

plot(1:size(df1),smooth(df1,10))

figure 

fm2=table2array(test2)-100

fm2=nonzeros(fm2)

fm2=fm2(fm1>=0)

df2=diff(fm2+1)

plot(1:size(df1),smooth(df1,10))

test2
test3




