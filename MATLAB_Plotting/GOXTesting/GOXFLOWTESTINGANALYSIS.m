PT1=[152.52 165.99 174.22 233.21 249.60];
PT2=[159.03 173.3 182.03 243.66 260.72];
figure


scatter(PT1,PT2)

fit=polyfit(PT1,PT2,1)


%%

close all
clear test

name={'test3-1.mat' 'test3-3.mat' 'test3-4.mat' 'test3-5.mat' 'test6-1.mat' 'test6-2.mat' 'test6-3.mat' 'test6-4.mat' 'test6-5.mat'  'test8-2.mat' 'test8-3.mat' 'test8-4.mat' 'test10-1.mat' 'test10-2.mat' 'test10-3.mat'};
for i=[1:size(name,2)]
    indata=load(name{i});
    imdata=indata.testData;
test(i).name=name{i};
test(i).pt1=imdata(:,1);
test(i).pt2=imdata(:,2);
test(i).fm=imdata(:,3);
test(i).time=imdata(:,4);
test(i).pt1raw=(test(i).pt1-20.8469)/(1.0533*10^(-4));
test(i).pt2raw=(test(i).pt2-17.9758)/(1.0323*10^(-4));

end


%%
%PT CALIBRATION 
close all

test(1).calpts=[200 600 700 800 900 1000 1100];
test(2).calpts=[100 200 400 500 600 700];
test(3).calpts=[100 200 300 400 700 800 900 1000];
test(4).calpts=[100 200 300 800 900 1000];
test(5).calpts=[100 200 300 400 500 600 700 800 1200 1300 1400];
test(6).calpts=[100 200 300 500 600 700];
test(7).calpts=[100 200 300 400 1200 1300 1400];
test(8).calpts=[100 200 500 600 700 800];
test(9).calpts=[100 200 400 500 600];
test(10).calpts=[100 200 300 600 800 900 1000];
test(11).calpts=[100 200 300 400 500 900 1000 1100 1200 1300 1400];
test(12).calpts=[100 200 300 400 700 800 900 1000 1100];
test(13).calpts=[100 200 400 600 800 1000];
test(14).calpts=[100 200 450 700 800];
test(15).calpts=[500 800 1100 1430];

figure
hold on
ptcal2.pt1=[];
ptcal2.pt2=[];
for i=[1:size(name,2)]

    ptcal(i).fit=polyfit(test(i).pt1raw(test(i).calpts),test(i).pt2raw(test(i).calpts),1);
  
    scatter(test(i).pt1raw(test(i).calpts),test(i).pt2raw(test(i).calpts));
    ptcal2.pt1=[ptcal2.pt1;test(i).pt1raw(test(i).calpts)];
   ptcal2.pt2=[ptcal2.pt2;test(i).pt2raw(test(i).calpts)];
   plot([0,3*10^6],[0,3*10^6]* ptcal(i).fit(1) + ptcal(i).fit(2));
end

ptcal(i).fit=polyfit(ptcal2.pt1,ptcal2.pt2,1)

%%
%Relative Calibrated PTs

%%
close all

for i=[1:size(name,2)]


%data2cal=data2;
%deltap=data1-data2cal;
%plot(smooth(deltap))
pt1plot=test(i).pt1;
pt2plot=test(i).pt2;

figure('Name',test(i).name)
subplot(3,1,1)
hold on

plot((pt1plot) )
plot((pt2plot  ))
title('Wrong Calibration PT Readings')
legend('PT1','PT2')

%subplot(3,1,2)
%hold on

pt1orig=smooth(test(i).pt1raw* ptcal(i).fit(1) + ptcal(i).fit(2));
D_npt=.5*25.6/10;
A_npt=pi()*(D_npt/4)^.5;%npt area in meters 

goxCal.fm_crit=[20.32 17.99 0 19.84 18.79 0 0 0 19.36 0 0 0 17.74 17.99 18.47]*.001;
goxCal.upper_p=[448.36 454.24 0 473.23 469.34 0 0 0 467.08 0 0 0 411.04 379.44 388.6];
goxCal.press_diff=[0 9.8 0 9.74 4.4 0 0 0 0 0 0 0 0 35 25]*6895;
%rho calculated at 20 C on peace software
goxCal.rho=[35.77 36.24 0 37.76 37.45 0 0 0 37.27 0 0 0 32.78 30.26 30.99];

%pt2orig=smooth(test(i).pt2raw-.5.*goxCal.rho(i).*goxCal.fm_crit(i)./(goxCal.rho(i).*A_npt*6895));
pt2orig=smooth(test(i).pt2raw);

%plot((pt1orig)  )
%plot((pt2orig ))
%title('Original PT Readings')
%legend('PT1','PT2')

%subplot(3,1,3)

ptorigdiff=pt1orig-pt2orig;
%plot((ptorigdiff)  )
%title('Original PT Difference')
%legend('Difference')

subplot(3,1,2)

test(i).ptdiffcal=smooth(pt1orig*1.0323*10^(-4)-(pt2orig*1.0323*10^(-4)),40);
plot((test(i).ptdiffcal)  )
title('Original PT Calibrated Difference')
legend('Difference')


subplot(3,1,3)
fm=test(i).fm;
plot(fm)
title('Mass Flow Meter Readings')

end

%%
close all
test.upstream=[445 451 470 471 465 467 461 459 460 383 391 380 387 372 381];
test.close=[353 360 320 320 319 377 371 319 319 0 0 0 262 250 256];
relief.closequarterhigh=test.close([1,2,6,7]);
relief.closequarterlow=test.close([3,4,5,8,9]);
relief.closehalf=test.close([13,14,15]);
relief.closequarterhighAVG=mean(relief.closequarterhigh);
relief.closequarterhighSTD=std(relief.closequarterhigh);
relief.closequarterlowAVG=mean(relief.closequarterlow);
relief.closequarterlowSTD=std(relief.closequarterlow);
relief.closehalfAVG=mean(relief.closehalf);
relief.closehalfSTD=std(relief.closehalf);


figure
scatter([1:15],relief.upstream)
figure
scatter([1:15],relief.close)
title('Pressure after relief valve close')
ylabel('Pressure [psi]')

%%
%Find pressure differences for smaller injector test where pressure
%difference is readily aparent. 


A=[0.0000112 0.0000112 0.0000112 0.0000112 0.0000112 0.0000262 0.0000262 0.0000262 0.0000262 0.0000262 0.0000262 0.0000262 0.0000112 0.0000112 0.0000112];
C_d=.56;
goxCal.B=goxCal.fm_crit./(C_d*A.* (2*goxCal.rho.*goxCal.press_diff).^(.5) );

goxCal.C_DCal=(goxCal.fm_crit)./(A.* (2*goxCal.rho.*(goxCal.press_diff)).^(.5) );

goxCal.B=goxCal.B([14,15])
goxCal.CD=goxCal.C_DCal([2,4,5,14,15])
B_const_avg=mean(goxCal.B)
Cd_avg=mean(goxCal.CD)