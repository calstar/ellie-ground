close all
data2cal=data2+.5;
deltap=data1-data2cal;
plot(smooth(deltap))
figure
hold on
plot(smooth(data1))
plot(smooth(data2cal))

legend('PT1','PT2')

figure

plot(smooth(data6))



