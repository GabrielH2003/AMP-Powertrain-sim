function st = plot_volta(out, pastaSaida)
%PLOT_VOLTA  Lap figure of pwtsimplificado (pure-pursuit driver on the FSAE track).
%   st = plot_volta(out)              -> saves docs/img/lap_map.png, returns lap stats
%   out = sim('pwtsimplificado')      (steer_mode = 5, piloto_on = 1, StopTime >= 30)
if nargin < 2
    pastaSaida = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'docs', 'img');
end
if ~exist(pastaSaida, 'dir'), mkdir(pastaSaida); end
S = load('pista_amp226.mat'); p = S.pista;

ls  = out.logsout;
P   = ls.getElement('pose3d').Values;           % [X Y Z roll pitch yaw delta thR thF]
t   = P.Time;  X = P.Data(:,1);  Y = P.Data(:,2);
k   = 1:200:numel(t);                            % 50 Hz is plenty for a map
t = t(k); X = X(k); Y = Y(k);
rs  = @(n) interp1(ls.getElement(n).Values.Time, squeeze(ls.getElement(n).Values.Data), t);
v   = interp1(out.get('Velocity').Time, squeeze(out.get('Velocity').Data), t) * 3.6;
Te  = interp1(out.get('Te').Time, squeeze(out.get('Te').Data), t);
Idc = rs('I_dc');  Brk = rs('Brake_ped');

% lap detection: project on the centre line and unwrap the arc length
[~, idx] = min(hypot(X - p.x', Y - p.y'), [], 2);
s  = unwrap(p.s(idx) / p.Lap * 2*pi) / (2*pi) * p.Lap;
s  = s - s(1);
iL = find(s >= p.Lap, 1);
if isempty(iL), iL = numel(t); end
d  = [0; cumsum(hypot(diff(X), diff(Y)))];
st = struct('t_volta', t(iL), 'distancia', d(iL), 'v_media', d(iL)/t(iL)*3.6, ...
            'v_max', max(v(1:iL)), 'Idc_pico', max(Idc(1:iL)), 'Idc_regen', min(Idc(1:iL)));
fprintf('lap %.2f s | %.1f m | mean %.1f km/h | max %.1f km/h | I_dc %.0f / %.0f A\n', ...
        st.t_volta, st.distancia, st.v_media, st.v_max, st.Idc_pico, st.Idc_regen);

c2 = [0.72 0.46 0.01]; c1 = [0.11 0.42 0.52]; c4 = [0.70 0.15 0.15]; cg = [0.6 0.6 0.6];
f = figure('Color','w','Position',[80 80 1300 640],'Visible','off');
tl = tiledlayout(f, 3, 2, 'TileSpacing','compact','Padding','compact');
title(tl, sprintf('Autonomous lap of the FSAE autocross track: %.2f s, %.0f m, top speed %.0f km/h', ...
      st.t_volta, st.distancia, st.v_max), 'FontSize', 13, 'FontWeight','bold');

ax = nexttile(tl, 1, [3 1]); hold(ax,'on'); axis(ax,'equal'); box(ax,'off');
plot(ax, p.cones_azul(:,1), p.cones_azul(:,2), '^', 'MarkerSize', 4, 'Color', [0.15 0.3 0.8], 'MarkerFaceColor', [0.15 0.3 0.8])
plot(ax, p.cones_amarelo(:,1), p.cones_amarelo(:,2), '^', 'MarkerSize', 4, 'Color', [0.85 0.7 0.05], 'MarkerFaceColor', [0.95 0.8 0.1])
plot(ax, p.cones_laranja_grande(:,1), p.cones_laranja_grande(:,2), '^', 'MarkerSize', 7, 'Color', [0.9 0.45 0.1], 'MarkerFaceColor', [0.95 0.5 0.1])
surface(ax, [X(1:iL) X(1:iL)], [Y(1:iL) Y(1:iL)], zeros(iL,2), [v(1:iL) v(1:iL)], ...
        'EdgeColor','interp','FaceColor','none','LineWidth',3);
colormap(ax, turbo); cb = colorbar(ax, 'eastoutside'); cb.Label.String = 'Speed (km/h)';
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
axis(ax,'tight'); xl = xlim(ax); yl = ylim(ax); xlim(ax, xl + [-4 4]); ylim(ax, yl + [-4 4]);

a1 = nexttile(tl, 2); plot(a1, d, v, 'Color', c1, 'LineWidth', 1.4); grid(a1,'on'); box(a1,'off')
ylabel(a1, 'Speed (km/h)'); xline(a1, d(iL), ':', 'lap', 'Color', cg)
a2 = nexttile(tl, 4); hold(a2,'on'); grid(a2,'on'); box(a2,'off')
plot(a2, d, Te, 'Color', c2, 'LineWidth', 1.2); plot(a2, d, Brk*100, 'Color', c4, 'LineWidth', 1.0)
yline(a2, 0, 'Color', cg); ylabel(a2, 'Torque (N·m) / brake (%)')
legend(a2, {'motor torque (< 0 = regen)', 'friction brake pedal'}, 'Location','northoutside','Orientation','horizontal','Box','off')
a3 = nexttile(tl, 6); plot(a3, d, Idc, 'Color', c1, 'LineWidth', 1.2); grid(a3,'on'); box(a3,'off')
yline(a3, 0, 'Color', cg); ylabel(a3, 'Battery current (A)'); xlabel(a3, 'Distance (m)')
linkaxes([a1 a2 a3], 'x'); xlim(a1, [0 d(end)])
exportgraphics(f, fullfile(pastaSaida, 'lap_map.png'), 'Resolution', 150); close(f)
end
