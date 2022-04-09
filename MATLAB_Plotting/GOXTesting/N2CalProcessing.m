close all
data1=testData(:,1);
data2=testData(:,2);


data2cal=data2;
deltap=data1-data2cal;
plot(smooth(deltap))
figure
hold on


plot(smooth((data1) ))
plot(smooth((data2cal  )))
title('Original PT Readings')
legend('PT1','PT2')

figure
hold on
plot(smooth((data1) * 1.0467 +.4648  ))
plot(smooth((data2cal  )))
title('Corrected PT Readings')
legend('PT1','PT2')

figure

plot(smooth(data6))



