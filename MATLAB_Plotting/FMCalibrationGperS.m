
%tank volume in cubic meters
V=4391e-6;

%molar mass of nitorgen g/mol
u=28;

%ideal gas constant J/mol K
R=8.314;

T=20+273; %K
InitialPressure=[101 115.7 147.4];
Changeinpressure=[15.6 14.7 19.5]*6895; %Pa
Cyclenumber=[71 68 88];
OpenTime=[.623 .603 .794]; %s

Cyclespersecond=Cyclenumber./OpenTime
ChangeinPressurePerTime=Changeinpressure./OpenTime

Constantgs=Changeinpressure.*V./(R/u*T.*OpenTime)
%Pa*m^3  /  [J/molK / (g/mol)]*K * s)
%N/m^2 *m^3 / (Nm/molK / (g/mol)*K )*s)
%Nm/ ( Nm/(gK))*K)*s
%g/s

Constantgperpulse=Changeinpressure.*V./(R/u*T.*Cyclenumber)

gramsPerGallon=3785;

pulsespergallon=gramsPerGallon./Constantgperpulse




