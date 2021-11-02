Data = Pyro18;
KZ_Rel = Data.KippAndZonenWm2./max(Data.KippAndZonenWm2);
Lux_Rel = Data.Luxlx./max(Data.Luxlx);
UVA_Rel = Data.UVA./max(Data.UVA);
UVB_Rel = Data.UVB./max(Data.UVB);
IRS_Rel = Data.IRShort./max(Data.IRShort);
IRM_Rel = Data.IRMid./max(Data.IRMid);

figure(1);
plot(KZ_Rel);
hold on
plot(Lux_Rel);
plot(UVA_Rel);
plot(UVB_Rel);
plot(IRS_Rel);
plot(IRM_Rel);

legend("CMP3", "Lux", "UVA", "UVB", "IR Short", "IR Mid");
title("Relative Full Band Monarch Data");
ylabel("Normalized Amplitude [0~1]");
xlabel("Samples");
set(gcf, 'Position', [0, 0, 1200, 500])

figure(2);
UV_Ratio = UVB_Rel./Lux_Rel;
plot(UV_Ratio./max(UV_Ratio));
hold on
plot(KZ_Rel);
legend("UVB Ratio", "CMP3");
title("UVB/Lux vs. CMP3");
ylabel("Normalized Amplitude [0~1]");
xlabel("Samples");
set(gcf, 'Position', [0, 0, 1200, 500])

figure(3);
IR_Ratio = IRS_Rel./Lux_Rel;
plot(IR_Ratio./max(IR_Ratio));
hold on
plot(KZ_Rel);
legend("IR Short Ratio", "CMP3");
title("IR Short/Lux vs. CMP3");
ylabel("Normalized Amplitude [0~1]");
xlabel("Samples");
set(gcf, 'Position', [0, 0, 1200, 500])

figure(4);
plot(IR_Ratio./max(IR_Ratio));
hold on
plot(UV_Ratio./max(UV_Ratio));
%plot(IR_Ratio./UV_Ratio);
legend("IR Short Ratio", "UVB Ratio");
title("IR Short/Lux vs. UVB/Lux");
ylabel("Normalized Amplitude [0~1]");
xlabel("Samples");
set(gcf, 'Position', [0, 0, 1200, 500])
