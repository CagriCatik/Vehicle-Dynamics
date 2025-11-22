function demo_ui()
%DEMO_UI Interactive demo: front steer and velocity time series.

    p = VehicleParams();

    x0 = 0.0;
    y0 = 0.0;
    yaw0 = 0.0;
    front_deg0 = 0.0;
    v0 = 0.0;

    sample_hz = 10.0;
    dt_sec = 1.0 / sample_hz;
    window_sec = 30.0;

    % Preallocate history (will grow up to maxlen)
    maxlen = floor(window_sec * sample_hz) + 5;
    t_hist = zeros(1, maxlen);
    f_hist = zeros(1, maxlen);
    v_hist = zeros(1, maxlen);
    hist_idx = 0;

    % Figure and layout
    fig = figure('Name', 'Planar Vehicle Demo', ...
                 'NumberTitle', 'off', ...
                 'Units', 'normalized', ...
                 'Position', [0.1 0.1 0.8 0.8]);

    % Use subplot layout similar to 3x2 grid
    ax_main = subplot(3, 2, 1, 'Parent', fig);
    ax_ts_f = subplot(3, 2, 3, 'Parent', fig);
    ax_ts_v = subplot(3, 2, 5, 'Parent', fig);

    info_ax  = subplot(3, 2, 2, 'Parent', fig);
    ax_front = subplot(3, 2, 4, 'Parent', fig);
    ax_vel   = subplot(3, 2, 6, 'Parent', fig);

    % Info axes
    axis(info_ax, 'off');

    % Make right-column axes framed but otherwise empty (we will place sliders over them)
    for a = [ax_front, ax_vel]
        axes(a); %#ok<LAXES>
        cla(a);
        axis(a, 'off');
        set(a, 'Box', 'on');
    end

    % Main axes configuration
    fixed_axes(ax_main, p);

    % Initial vehicle polygons at origin
    [body, fr, fl, rr, rl] = vehicle_parts(p, yaw0, deg2rad(front_deg0));
    off = [x0; y0];
    body = body + off;
    fr   = fr   + off;
    fl   = fl   + off;
    rr   = rr   + off;
    rl   = rl   + off;

    colors = {'k', [0 0.4470 0.7410], [0 0.4470 0.7410], ...
                 [0.8500 0.3250 0.0980], [0.8500 0.3250 0.0980]};
    polys  = {body, fr, fl, rr, rl};
    patches = gobjects(1, numel(polys));

    axes(ax_main); %#ok<LAXES>
    hold(ax_main, 'on');
    for i = 1:numel(polys)
        poly = polys{i};
        col  = colors{i};
        patches(i) = patch(poly(1,:), poly(2,:), 'w', ...
                           'EdgeColor', col, ...
                           'LineWidth', 2, ...
                           'FaceColor', 'none', ...
                           'Parent', ax_main);
    end

    % Arrow graphics
    arrow_color = [0.0 0.6 0.0];
    L = 0.6 * p.wheelbase;

    shaft = plot(ax_main, NaN, NaN, 'LineWidth', 2, ...
                 'Color', arrow_color, 'LineCap', 'round');
    head_l = plot(ax_main, NaN, NaN, 'LineWidth', 2, 'Color', arrow_color);
    head_r = plot(ax_main, NaN, NaN, 'LineWidth', 2, 'Color', arrow_color);

    set_arrow(x0, y0, yaw0);

    % Info text
    info_txt = text(info_ax, 0.5, 0.5, '', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'FontName', 'monospaced', ...
        'FontSize', 10, ...
        'Units', 'normalized', ...
        'BackgroundColor', 'w', ...
        'EdgeColor', 0.6 * [1 1 1]);

    update_info(front_deg0, v0);

    % Sliders (uicontrol), placed roughly over the right column
    front_slider = uicontrol('Style', 'slider', ...
        'Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [0.78 0.40 0.04 0.40], ...
        'Min', -30.0, 'Max', 30.0, 'Value', front_deg0, ...
        'SliderStep', [0.1/60, 1/60], ...
        'Callback', @on_slider_change);

    front_label = uicontrol('Style', 'text', ...
        'Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [0.76 0.81 0.08 0.05], ...
        'String', 'Front steer (deg)', ...
        'HorizontalAlignment', 'center');

    vel_slider = uicontrol('Style', 'slider', ...
        'Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [0.90 0.40 0.04 0.40], ...
        'Min', 0.0, 'Max', 30.0, 'Value', v0, ...
        'SliderStep', [0.1/30, 1/30], ...
        'Callback', @on_slider_change);

    vel_label = uicontrol('Style', 'text', ...
        'Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [0.88 0.81 0.08 0.05], ...
        'String', 'Velocity (m/s)', ...
        'HorizontalAlignment', 'center');

    % Time-series axes styling
    style_ts(ax_ts_f, 'Front steer (deg)');
    style_ts(ax_ts_v, 'Velocity (m/s)', 'Time (s)');

    % Secondary y-axis for km/h
    ax_ts_v_kmh = axes('Position', get(ax_ts_v, 'Position'), ...
                       'Color', 'none', ...
                       'YAxisLocation', 'right', ...
                       'XAxisLocation', 'top', ...
                       'XColor', 'none', ...
                       'YColor', 'k');
    linkaxes([ax_ts_v, ax_ts_v_kmh], 'x');
    ylabel(ax_ts_v_kmh, 'Velocity (km/h)');
    set(ax_ts_v_kmh, 'XTick', [], 'Box', 'off');

    % Time-series lines
    axes(ax_ts_f); %#ok<LAXES>
    ln_f = plot(ax_ts_f, NaN, NaN, 'LineWidth', 1.8);

    axes(ax_ts_v); %#ok<LAXES>
    ln_v = plot(ax_ts_v, NaN, NaN, 'LineWidth', 1.8);

    t0 = tic;

    % Timer for sampling and drawing
    timerObj = timer('ExecutionMode', 'fixedRate', ...
                     'Period', dt_sec, ...
                     'TimerFcn', @sample_and_draw);

    start(timerObj);

    % Close callback to clean up timer
    set(fig, 'CloseRequestFcn', @on_close);

    % Nested helper functions

    function on_slider_change(~, ~)
        f_deg = get(front_slider, 'Value');
        v = get(vel_slider, 'Value');

        % Update vehicle shape
        [b, f_r, f_l, r_r, r_l] = vehicle_parts(p, yaw0, deg2rad(f_deg));
        off2 = [x0; y0];
        polys2 = {b + off2, f_r + off2, f_l + off2, r_r + off2, r_l + off2};

        for ii = 1:numel(patches)
            set(patches(ii), 'XData', polys2{ii}(1,:), ...
                             'YData', polys2{ii}(2,:));
        end

        % Arrow
        set_arrow(x0, y0, yaw0);

        % Info text
        update_info(f_deg, v);

        drawnow limitrate;
    end

    function update_info(front_deg, vel_mps)
        set(info_txt, 'String', sprintf('front: %5.1f deg\nvelocity: %6.1f km/h', ...
                                        front_deg, vel_mps * 3.6));
    end

    function set_arrow(x, y, theta)
        ang = deg2rad(30.0);
        x_end = x + L * cos(theta);
        y_end = y + L * sin(theta);

        set(shaft, 'XData', [x, x_end], ...
                   'YData', [y, y_end]);

        hL = 0.4 * L;
        lt = theta + pi - ang;
        rt = theta + pi + ang;

        lx = x_end + hL * cos(lt);
        ly = y_end + hL * sin(lt);
        rx = x_end + hL * cos(rt);
        ry = y_end + hL * sin(rt);

        set(head_l, 'XData', [x_end, lx], 'YData', [y_end, ly]);
        set(head_r, 'XData', [x_end, rx], 'YData', [y_end, ry]);
    end

    function style_ts(ax, ylabel_str, xlabel_str)
        if nargin < 3
            xlabel_str = '';
        end
        axes(ax); %#ok<LAXES>
        grid(ax, 'on');
        ax.GridLineStyle = '--';
        ax.GridAlpha = 0.4;
        ylabel(ax, ylabel_str);
        if ~isempty(xlabel_str)
            xlabel(ax, xlabel_str);
        end
        xlim(ax, [0, window_sec]);
    end

    function auto_ylim(ax, data, pad, min_span)
        if isempty(data)
            return;
        end
        lo = min(data);
        hi = max(data);
        if hi - lo < min_span
            mid = 0.5 * (hi + lo);
            lo = mid - 0.5 * min_span;
            hi = mid + 0.5 * min_span;
        end
        span = hi - lo;
        ylim(ax, [lo - pad * span, hi + pad * span]);
    end

    function sample_and_draw(~, ~)
        t = toc(t0);
        f_deg = get(front_slider, 'Value');
        v = get(vel_slider, 'Value');

        % Update histories
        hist_idx = hist_idx + 1;
        if hist_idx > maxlen
            hist_idx = maxlen;
            % Shift data left
            t_hist(1:end-1) = t_hist(2:end);
            f_hist(1:end-1) = f_hist(2:end);
            v_hist(1:end-1) = v_hist(2:end);
        end

        t_hist(hist_idx) = t;
        f_hist(hist_idx) = f_deg;
        v_hist(hist_idx) = v;

        % Valid data indices within time window
        t_arr = t_hist(1:hist_idx);
        idx = t_arr >= max(0.0, t - window_sec);
        t_win = t_arr(idx);

        if isempty(t_win)
            x_plot = [];
        else
            x_plot = t_win - t_win(1);
        end

        f_arr = f_hist(1:hist_idx);
        v_arr = v_hist(1:hist_idx);

        set(ln_f, 'XData', x_plot, ...
                  'YData', f_arr(idx));
        set(ln_v, 'XData', x_plot, ...
                  'YData', v_arr(idx));

        % Adjust x-limits
        xlim(ax_ts_f, [0, window_sec]);
        xlim(ax_ts_v, [0, window_sec]);

        % Adjust y-limits
        auto_ylim(ax_ts_f, f_arr(idx), 0.15, 2.0);
        auto_ylim(ax_ts_v, v_arr(idx), 0.15, 0.5);

        v_lim = ylim(ax_ts_v);
        set(ax_ts_v_kmh, 'YLim', v_lim * 3.6);

        drawnow limitrate;
    end

    function on_close(~, ~)
        try
            stop(timerObj);
            delete(timerObj);
        catch
        end
        delete(fig);
    end
end
