classdef VehicleParams
    % VehicleParams: geometric parameters for planar vehicle model.

    properties
        wheelbase double = 2.9
        width double = 2.0
        length double = 4.9
        tire_radius double = 0.4  % used as half wheel LENGTH proxy in plan view
        tire_width double = 0.3   % used as wheel WIDTH in plan view
        wheel_track double = 1.8  % full distance between wheel centers
        rear_overhang double = 1.0
        front_overhang double = 1.0
    end

    properties (Dependent)
        half_track   % 0.5 * wheel_track
        wheel_len    % 2 * tire_radius
        wheel_w      % tire_width
    end

    methods
        function obj = VehicleParams(varargin)
            % Optional name/value pairs, e.g. VehicleParams("wheelbase", 3.0)
            if mod(nargin, 2) ~= 0
                error("VehicleParams: constructor expects name/value pairs.");
            end
            for k = 1:2:nargin
                name = varargin{k};
                val  = varargin{k+1};
                if isprop(obj, name)
                    obj.(name) = val;
                else
                    error("VehicleParams: unknown property %s.", name);
                end
            end
            obj = obj.validate_params();
        end

        function v = get.half_track(obj)
            v = 0.5 * obj.wheel_track;
        end

        function v = get.wheel_len(obj)
            v = 2.0 * obj.tire_radius;
        end

        function v = get.wheel_w(obj)
            v = obj.tire_width;
        end

        function obj = validate_params(obj)
            % Match overall length with overhangs + wheelbase.
            expected_len = obj.rear_overhang + obj.wheelbase + obj.front_overhang;
            tol = max(1e-6 * abs(expected_len), 1e-9);
            if abs(obj.length - expected_len) > tol
                obj.length = expected_len;
            end
        end
    end
end
