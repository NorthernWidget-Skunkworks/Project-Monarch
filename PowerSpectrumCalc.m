SamplePoints = linspace(250, 2500, 1000);
Response = interp1(VEMD1060X01IRResponse.x, VEMD1060X01IRResponse.Curve1, SamplePoints, 'nearest', 0);
Solar = interp1(SolarPower.x, SolarPower.Curve1, SamplePoints, 'nearest', 0);

Power = 0;
Count = 0;

for i = 2:length(SamplePoints)
    if(SolarPower.x(i) < StopWavelength && SolarPower.x(i) > StartWavelength)
        Count = Count + 1;
        Power = Power + SolarPower.Curve1(i)*(SolarPower.x(i) - SolarPower.x(i-1));
    end
% Power = Power + (Solar(i)*(SamplePoints(i) - SamplePoints(i-1)))*Response(i);
end

Power