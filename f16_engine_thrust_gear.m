function POW = f16_engine_thrust_gear(THROTTLE)

if THROTTLE <= 0.77 
    POW = 64.94*THROTTLE;
else % afterburning
    POW = 217.38*THROTTLE - 117.38;
end

end