% --------------------------------------------------------------------
%  File: PacejkaTire.m
%  Description: Object‑oriented implementation of the script’s model.
% --------------------------------------------------------------------
classdef PacejkaTire
    properties
        % vehicle
        mass
        g
        static_wdist
        % tire
        par   % struct holding all magic‑formula coefficients
    end
    methods
        %% constructor
        function obj = PacejkaTire(params)
            if nargin==0
                params = obj.defaultParams();
            end
            obj.par = params;
            obj.mass = 1500;
            obj.g = 9.81;
            obj.static_wdist = 0.6;
        end

        %% ----------------------------------------------------------------
        %  Pure‑slip forces
        function Fx = calcFx0(obj, kappa, Fz)
            B = obj.Kx_small(Fz)./(obj.par.Cx .* (obj.mu_x(Fz).*Fz));
            Fx = obj.mf(kappa, B, obj.par.Cx, obj.mu_x(Fz).*Fz, obj.par.Ex, 0, 0);
        end

        function Fy = calcFy0(obj, alpha, Fz, gamma)
            B = obj.Ca_small(Fz)./(obj.par.Cy .* (obj.mu_y(Fz).*Fz));
            Fy = obj.mf(alpha, B, obj.par.Cy, obj.mu_y(Fz).*Fz, obj.par.Ey,...
                      obj.par.Sh_y_gamma*gamma, obj.par.Sv_y_gamma*gamma*(Fz/obj.Fz_nom)*Fz);
        end
        % ----------------------------------------------------------------
        %  Combined‑slip forces
        function Fx = calcFx(obj, kappa, alpha, Fz, gamma)
            Fx = obj.calcFx0(kappa, Fz).*obj.Gx(alpha, Fz);
        end

        function Fy = calcFy(obj, alpha, kappa, Fz, gamma)
            Fy = obj.calcFy0(alpha, Fz, gamma).*obj.Gy(kappa, Fz);
        end
        % ----------------------------------------------------------------
        %  Aligning moment
        function Mz = calcMz(obj, alpha, kappa, Fz, gamma)
            trail = obj.trail_comb(alpha, kappa, Fz);
            Mz = -trail .* obj.calcFy(alpha, kappa, Fz, gamma);
        end
        % ----------------------------------------------------------------
        %  Helpers
        function B = mu_x(obj, Fz)
            B = obj.par.mu_x0 .* (1+obj.par.mu_x_load_slope*((Fz/obj.Fz_nom)-1));
        end
        function B = mu_y(obj, Fz)
            B = obj.par.mu_y0 .* (1+obj.par.mu_y_load_slope*((Fz/obj.Fz_nom)-1));
        end
        function B = Kx_small(obj, Fz)
            B = obj.par.Kx0 .* (Fz/obj.Fz_nom).^obj.par.Kx_exp;
        end
        function B = Ca_small(obj, Fz)
            B = obj.par.Ca0 .* (Fz/obj.Fz_nom).^obj.par.Ca_exp;
        end
        function Gx = Gx(obj, alpha, Fz)
            Gx = cos(obj.par.Cgx .* atan(obj.Bgx(Fz).*alpha));
        end
        function Gy = Gy(obj, kappa, Fz)
            Gy = cos(obj.par.Cgy .* atan(obj.Bgy(Fz).*kappa));
        end
        function B = Bgx(obj, Fz)
            B = max(0.1, obj.par.Bgx0 .* (obj.Fz_nom./Fz).^obj.par.Bg_exp);
        end
        function B = Bgy(obj, Fz)
            B = max(0.1, obj.par.Bgy0 .* (obj.Fz_nom./Fz).^obj.par.Bg_exp);
        end
        function t = trail0(obj, Fz)
            t = obj.par.t0 .* (Fz/obj.Fz_nom).^obj.par.t_exp;
        end
        function t = trail_pure(obj, alpha, Fz)
            t = obj.trail0(Fz) .* cos(obj.par.Ct .* atan(obj.Bt(Fz).*alpha));
        end
        function t = trail_comb(obj, alpha, kappa, Fz)
            t = obj.trail_pure(alpha, Fz) .* ...
                cos(obj.par.Ct .* atan(obj.par.Gt_kappa .* obj.Btk(Fz).*kappa));
        end
        function B = Bt(obj, ~)
            B = obj.par.Bt0;
        end
        function B = Btk(obj, ~)
            B = obj.par.Btk0;
        end
        function y = mf(obj, x,B,C,D,E,Sh,Sv)
            y = D .* sin(C .* atan(B.*(x+Sh) - E.*(B.*(x+Sh)-atan(B.*(x+Sh))))) + Sv;
        end
        % ----------------------------------------------------------------
        function params = defaultParams(~)
            % Return a struct with the same field layout as the script
            params.mu_x0 = 1.00;  % … etc. (copy all from the script)
        end
    end
end