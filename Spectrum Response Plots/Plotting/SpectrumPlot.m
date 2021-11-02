
figure(1)
% semilogx(VEML6030WhiteResponse.x, VEML6030WhiteResponse.Curve1)
% hold on
% semilogx(VEML6075UVResponse.x, VEML6075UVResponse.UVA)
% semilogx(VEML6075UVResponse.x, VEML6075UVResponse.UVB)
% semilogx(VEML6030FilteredResponse.x, VEML6030FilteredResponse.Curve1)
% semilogx(IRMidWaveResponse.x, IRMidWaveResponse.Curve1./0.9571)
% semilogx(IRShortWaveResponse.x, IRShortWaveResponse.Curve1./0.5654)

plot(VEML6030WhiteResponse.x, VEML6030WhiteResponse.Curve1)
hold on
plot(VEML6075UVResponse.x, VEML6075UVResponse.UVA)
plot(VEML6075UVResponse.x, VEML6075UVResponse.UVB)
plot(VEML6030FilteredResponse.x, VEML6030FilteredResponse.Curve1)
plot(IRMidWaveResponse.x, IRMidWaveResponse.Curve1./0.9571)
plot(IRShortWaveResponse.x, IRShortWaveResponse.Curve1./0.5654)

% plot(ApogeeSP110Response.x, ApogeeSP110Response.Curve1)

title('Project Monarch Sub-sensor Spectral Response');
legend('WhiteBroad','UVA','UVB','WhiteFilt','IR-Mid','IR-Short')

ylabel('Spectral Sensitivity');
xlabel('Wavelength [nm]');
axis([250, 1000, 0, 1.2]);
xtickangle(45);

SamplePoints = linspace(200, 5000, 1000);

Broad = interp1(VEML6030WhiteResponse.x, VEML6030WhiteResponse.Curve1, SamplePoints);
UVA = interp1(VEML6075UVResponse.x, VEML6075UVResponse.UVA, SamplePoints);
UVB = interp1(VEML6075UVResponse.x, VEML6075UVResponse.UVB, SamplePoints);
BroadFilt = interp1(VEML6030FilteredResponse.x, VEML6030FilteredResponse.Curve1, SamplePoints);
IR_Mid = interp1(IRMidWaveResponse.x, IRMidWaveResponse.Curve1./0.9571, SamplePoints);
IR_Short = interp1(IRShortWaveResponse.x, IRShortWaveResponse.Curve1./0.5654, SamplePoints);
KippAndZonen = interp1(KippAndZonenCMP3Response.x, KippAndZonenCMP3Response.Curve1, SamplePoints);

Monarch = 1:1000;
for i = 1:1000
    Data = [Broad(i), UVA(i), UVB(i), BroadFilt(i), IR_Mid(i), IR_Short(i)];
    Monarch(i) = max(Data);
end
BlackBody = (2.*pi.*h.*(c.^2))./(((SamplePoints*1e-9).^5).*(exp((h.*c)./(k.*(SamplePoints*1e-9).*5800)) - 1));
BlackMax = max(BlackBody);
BlackBody = BlackBody./BlackMax;

figure(2)
subplot(2,1,1);
semilogx(SamplePoints, Monarch)
hold on
semilogx(ApogeeSP110Response.x, ApogeeSP110Response.Curve1)
semilogx(KippAndZonenCMP3Response.x, KippAndZonenCMP3Response.Curve1)
semilogx(SamplePoints, BlackBody);
axis([200, 3000, 0, 1.2]);
title('Broadband Spectral Response, Log');
ylabel('Spectral Sensitivity');
xlabel('Wavelength [nm]');

subplot(2,1,2);
Line1 = plot(SamplePoints, Monarch);
hold on
Line2 = plot(ApogeeSP110Response.x, ApogeeSP110Response.Curve1);
Line3 = plot(KippAndZonenCMP3Response.x, KippAndZonenCMP3Response.Curve1);
Line4 = plot(SamplePoints, BlackBody);
h = 4.135e-15;
c = 3e8;
k = 8.617e-5;


axis([200, 3000, 0, 1.2]);

title('Broadband Spectral Response, Linear');
legend('Project Monarch', 'Kipp and Zonen CMP3', 'Apogee SP110', 'Solar Black Body');
ylabel('Spectral Sensitivity');
xlabel('Wavelength [nm]');

SamplePoints_Long = linspace(2500, 35000, 1000);
BlackBody_Earth = (2.*pi.*h.*(c.^2))./(((SamplePoints_Long*1e-9).^5).*(exp((h.*c)./(k.*(SamplePoints_Long*1e-9).*288)) - 1));
BlackEarthMax = max(BlackBody_Earth);
BlackBody_Earth = BlackBody_Earth./BlackEarthMax;

IR_Long = interp1(IRLongWaveResponse.x, IRLongWaveResponse.Curve1./0.8764, SamplePoints_Long);
% KippAndZonen_Long = interp1(KippAndZonenCMP3Response.x, KippAndZonenCMP3Response.Curve1, SamplePoints_Long);

figure(3)
plot(SamplePoints_Long, BlackBody_Earth)
hold on
plot(SamplePoints_Long, IR_Long)
% plot(SamplePoints_Long, KippAndZonen_Long)
axis([2500, 35000, 0, 1.2]);
title('Long Wave Spectral Response');
legend('Project Monarch', 'Earth Black Body');
ylabel('Spectral Sensitivity');
xlabel('Wavelength [nm]');

figure(4)
semilogx(SamplePoints, Monarch)
hold on
PeakSampleValues = [28, 34, 75, 83, 138, 285];
for i = 1:6
    PeakSampleValues(i) = SamplePoints(PeakSampleValues(i));
    line([PeakSampleValues(i), PeakSampleValues(i)], [0, 1.2], 'Color', 'black', 'LineStyle', '--');
end
axis([250, 2000, 0, 1.2]);
xtickangle(45);
title('Project Monarch Spectral Sample Points');
ylabel('Spectral Sensitivity');
xlabel('Wavelength [nm]');
legend('Effective Response', 'Sample Peaks');
