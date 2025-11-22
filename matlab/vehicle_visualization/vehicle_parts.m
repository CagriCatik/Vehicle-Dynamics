function [body, fr, fl, rr, rl] = vehicle_parts(params, yaw, steer_front)
%VEHICLE_PARTS Planar vehicle geometry in world frame (row-rotated).
%   [body, fr, fl, rr, rl] = vehicle_parts(params, yaw, steer_front)
%   returns 2xN polygons (x; y) for the body and four wheels.
%
%   Angles in radians, CCW positive.

    % Ensure params length is consistent
    params = params.validate_params();

    body = body_polygon(params);
    wheel = wheel_polygon(params);

    % Front wheel steering
    R_f = R_row(steer_front);
    fr = apply_row_rot(wheel, R_f);
    fl = apply_row_rot(wheel, R_f);

    % Rear wheels not steered
    rr = wheel;
    rl = wheel;

    % Translate wheels in vehicle frame
    fr = fr + [params.wheelbase; -params.half_track];
    fl = fl + [params.wheelbase;  params.half_track];
    rr = rr + [0.0;              -params.half_track];
    rl = rl + [0.0;               params.half_track];

    % Apply body yaw to all parts (about origin)
    R_yaw = R_row(yaw);
    body = apply_row_rot(body, R_yaw);
    fr   = apply_row_rot(fr,   R_yaw);
    fl   = apply_row_rot(fl,   R_yaw);
    rr   = apply_row_rot(rr,   R_yaw);
    rl   = apply_row_rot(rl,   R_yaw);
end

function R = R_row(theta)
% Row-vector rotation: p' = p * R, CCW for +theta.
    c = cos(theta);
    s = sin(theta);
    R = [c,  s; ...
        -s,  c];
end

function poly_out = apply_row_rot(poly_in, R)
% poly_in: 2xN, poly_out: 2xN, row-vector rotation.
    poly_out = (poly_in.' * R).';
end

function body = body_polygon(p)
% 2x5 rectangle outline of vehicle body.
    x_front = p.wheelbase + p.front_overhang;
    x_rear  = -p.rear_overhang;
    y_half  = 0.5 * p.width;
    body = [ x_rear,  x_rear,  x_front, x_front, x_rear; ...
              y_half, -y_half, -y_half, y_half,  y_half ];
end

function wheel = wheel_polygon(p)
% Long in x (wheel_len), narrow in y (wheel_w).
    L = 0.5 * p.wheel_len;
    W = 0.5 * p.wheel_w;
    wheel = [ L,  -L,  -L,   L,   L; ...
             -W,  -W,   W,   W,  -W ];
end
