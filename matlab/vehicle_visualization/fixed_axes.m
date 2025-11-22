function fixed_axes(ax, params)
%FIXED_AXES Configure axes to fixed view around vehicle at origin.

    axes(ax); %#ok<LAXES>
    axis(ax, 'equal');
    daspect(ax, [1 1 1]);

    x_min = -params.rear_overhang - 0.6;
    x_max =  params.wheelbase + params.front_overhang + 0.6;
    y_half = 0.5 * params.width;

    xlim(ax, [x_min, x_max]);
    ylim(ax, [-y_half - 1.0, y_half + 1.4]);

    grid(ax, 'on');
    ax.GridLineStyle = '--';
    ax.GridAlpha = 0.4;

    title(ax, 'Planar Vehicle (front-steer only)');
end
