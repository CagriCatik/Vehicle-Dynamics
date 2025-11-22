function draw_vehicle(x, y, yaw, steer_front, params, varargin)
%DRAW_VEHICLE Draw planar vehicle with front steering only.
%
%   draw_vehicle(x, y, yaw, steer_front, params, ...
%                'color_front', [r g b] or 'b', ...
%                'color_rear', [r g b] or 'r', ...
%                'outline_color', 'k', ...
%                'arrow_color', [r g b] or 'g', ...
%                'color', [r g b] or 'k')
%
%   Angles in radians. Position in same units as params (typically meters).

    % Default options
    opts.color_front  = [0 0.4470 0.7410];    % similar to tab:blue
    opts.color_rear   = [0.8500 0.3250 0.0980]; % similar to tab:orange
    opts.outline_color = 'k';
    opts.arrow_color   = [];
    opts.color         = [];

    % Parse name/value pairs
    if ~isempty(varargin)
        if mod(numel(varargin), 2) ~= 0
            error('draw_vehicle: name/value arguments must come in pairs.');
        end
        for k = 1:2:numel(varargin)
            name = varargin{k};
            val  = varargin{k+1};
            if isfield(opts, name)
                opts.(name) = val;
            else
                error('draw_vehicle: unknown option "%s".', name);
            end
        end
    end

    % If "color" given, override all
    if ~isempty(opts.color)
        opts.outline_color = opts.color;
        opts.color_front   = opts.color;
        opts.color_rear    = opts.color;
    end

    % Geometry in world frame about origin
    [body, fr, fl, rr, rl] = vehicle_parts(params, yaw, steer_front);

    % Translate to (x, y)
    off = [x; y];
    body = body + off;
    fr   = fr   + off;
    fl   = fl   + off;
    rr   = rr   + off;
    rl   = rl   + off;

    hold_state = ishold;
    hold on;

    % Body and wheels
    plot(body(1,:), body(2,:), 'Color', opts.outline_color, 'LineWidth', 2);
    plot(fr(1,:),   fr(2,:),   'Color', opts.color_front,   'LineWidth', 2);
    plot(fl(1,:),   fl(2,:),   'Color', opts.color_front,   'LineWidth', 2);
    plot(rr(1,:),   rr(2,:),   'Color', opts.color_rear,    'LineWidth', 2);
    plot(rl(1,:),   rl(2,:),   'Color', opts.color_rear,    'LineWidth', 2);

    % Wheelbase line and rear axle marker
    xb = x + params.wheelbase * cos(yaw);
    yb = y + params.wheelbase * sin(yaw);
    plot([x, xb], [y, yb], '--', 'Color', 0.4 * [1 1 1], 'LineWidth', 1.2);
    plot(x, y, '*k');

    % Optional arrow
    if ~isempty(opts.arrow_color)
        L = 0.6 * params.wheelbase;
        draw_arrow(x, y, yaw, L, opts.arrow_color);
    end

    if ~hold_state
        hold off;
    end
end
