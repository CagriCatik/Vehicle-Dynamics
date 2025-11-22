% Basic static draw
p = VehicleParams();
figure;
draw_vehicle(0.0, 0.0, 0.0, deg2rad(15), p, ...
             'arrow_color', [0 0.6 0]);

% Interactive demo (analogous to __main__ in Python)
demo_ui();
