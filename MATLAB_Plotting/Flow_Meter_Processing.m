close all
figure
fm=table2array(fmtest2)-1

fm=nonzeros(fm)

df=diff(fm+1)

plot(1:size(df),smooth(df,10))

