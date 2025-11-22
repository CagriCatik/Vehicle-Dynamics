function draw_arrow(x, y, theta, length_val, color)
%DRAW_ARROW Simple arrow primitive (line + two head segments).
%   Angles in radians.

    ang = deg2rad(30.0);

    dx = length_val * cos(theta);
    dy = length_val * sin(theta);

    x_end = x + dx;
    y_end = y + dy;

    hold_state = ishold;
    hold on;

    plot([x, x_end], [y, y_end], 'Color', color, 'LineWidth', 2);

    hL = 0.4 * length_val;
    lt = theta + pi - ang;
    rt = theta + pi + ang;

    lx = x_end + hL * cos(lt);
    ly = y_end + hL * sin(lt);
    rx = x_end + hL * cos(rt);
    ry = y_end + hL * sin(rt);

    plot([x_end, lx], [y_end, ly], 'Color', color, 'LineWidth', 2);
    plot([x_end, rx], [y_end, ry], 'Color', color, 'LineWidth', 2);

    if ~hold_state
        hold off;
    end
end
