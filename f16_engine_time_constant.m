function RTAU = f16_engine_time_constant(DP)

if DP <= 25
    RTAU = 1.0;
elseif DP >= 50
    RTAU = 0.1;
else
    RTAU = 1.9 - 0.036*DP;
end
    

end