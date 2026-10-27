function plot_resultados(out, pastaSaida)
%PLOT_RESULTADOS  Figures of a full-throttle run of pwtsimplificado_longitudinal.
%   plot_resultados(out)              -> saves PNGs in docs/img
%   plot_resultados(out, pastaSaida)  -> saves PNGs in pastaSaida
%   out = output of sim('pwtsimplificado_longitudinal')
if nargin < 2
    pastaSaida = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'docs', 'img');
end
if ~exist(pastaSaida, 'dir'), mkdir(pastaSaida); end

g   = @(n) squeeze(out.get(n).Data);
t   = out.get('Velocity').Time;
v   = g('Velocity') * 3.6;                 % km/h
Te  = g('Te');  Tr = g('Torque_ref');
rpm = g('wm') * 60 / (2*pi);               % motor speed
Idm = g('Idm'); Iqm = g('Iqm'); Idr = g('Idref');
Vm  = g('Vmag');

% decimate for plotting (log is at 8 kHz)
k = 1:20:numel(t);
t = t(k); v = v(k); Te = Te(k); Tr = Tr(k); rpm = rpm(k);
Idm = Idm(k); Iqm = Iqm(k); Idr = Idr(k); Vm = Vm(k);

V_ref_fw = 168*0.95/sqrt(3)*0.95;          % field-weakening voltage reference
iK  = find(t > 0.5 & Vm >= V_ref_fw, 1);   % FW knee
tK  = t(iK);
x   = cumtrapz(t, v/3.6);
i60 = find(v >= 60, 1);  i75 = find(x >= 75, 1);

c1 = [0.11 0.42 0.52]; c2 = [0.72 0.46 0.01]; c3 = [0.55 0.55 0.55]; c4 = [0.70 0.15 0.15];
f = figure('Color','w','Position',[100 100 1100 760],'Visible','off');
tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
title(tl, 'Default scenario: full throttle from rest, braking 9.5–11.5 s, throttle again', ...
      'FontSize', 13, 'FontWeight', 'bold');

ax1 = nexttile; hold on; grid on; box off
plot(t, v, 'Color', c1, 'LineWidth', 1.8)
plot([t(i60) t(i60)], [0 60], ':', 'Color', c3); plot(t(i60), 60, 'o', 'Color', c1, 'MarkerFaceColor', c1)
text(t(i60)+0.1, 52, sprintf('0–60 km/h  %.2f s', t(i60)), 'FontSize', 10)
plot(t(i75), v(i75), 's', 'Color', c2, 'MarkerFaceColor', c2)
text(t(i75)+0.1, v(i75)-8, sprintf('75 m  %.2f s', t(i75)), 'FontSize', 10)
ylabel('Vehicle speed (km/h)'); xlim([0 t(end)])

ax2 = nexttile; hold on; grid on; box off
plot(t, Tr, '--', 'Color', c3, 'LineWidth', 1.2)
plot(t, Te, 'Color', c2, 'LineWidth', 1.6)
xline(tK, ':', sprintf('voltage limit reached, %.0f rpm', rpm(iK)), 'Color', c4, ...
      'LabelVerticalAlignment', 'bottom', 'FontSize', 9)
ylabel('Motor torque (N·m)'); legend({'request','electromagnetic'}, 'Location','southwest','Box','off')
xlim([0 t(end)])

ax3 = nexttile; hold on; grid on; box off
plot(t, Idr, '--', 'Color', c3, 'LineWidth', 1.2)
plot(t, Idm, 'Color', c1, 'LineWidth', 1.2)
plot(t, Iqm, 'Color', c2, 'LineWidth', 1.2)
xline(tK, ':', 'Color', c4)
ylabel('Current (A, peak)'); xlabel('Time (s)')
legend({'I_d reference','I_d','I_q'}, 'Location','northeast','Box','off'); xlim([0 t(end)])
linkaxes([ax1 ax2 ax3], 'x')
exportgraphics(f, fullfile(pastaSaida, 'acceleration_run.png'), 'Resolution', 150); close(f)

% ---- torque-speed envelope vs datasheet -------------------------------
ds_rpm = [2820 3310 3800 4290 4780];       % datasheet peak curve at 144 V
ds_T   = [94   80   68   55   46];
esc    = 2970/2820;                        % speed scaling to 168 V pack, Mod = 0.95
full   = Tr > 90 & t > 0.3 & t < 9.4;   % throttle phase only
f = figure('Color','w','Position',[100 100 900 520],'Visible','off'); hold on; grid on; box off
plot(rpm(full), Te(full), 'Color', c2, 'LineWidth', 2)
plot(ds_rpm, ds_T, 'o--', 'Color', c3, 'MarkerFaceColor', 'w')
plot(ds_rpm*esc, ds_T, 's-', 'Color', c1, 'MarkerFaceColor', c1)
xline(rpm(iK), ':', sprintf('model knee %.0f rpm', rpm(iK)), 'Color', c4, 'LabelVerticalAlignment','bottom')
xlabel('Motor speed (rpm)'); ylabel('Torque (N·m)'); ylim([0 105])
legend({'model, full throttle (168 V)', 'datasheet peak curve (144 V)', 'datasheet scaled to 168 V'}, ...
       'Location', 'southwest', 'Box', 'off')
title('Torque–speed envelope: model vs motor datasheet', 'FontSize', 12)
exportgraphics(f, fullfile(pastaSaida, 'torque_speed.png'), 'Resolution', 150); close(f)
fprintf('Figures saved in %s\n', pastaSaida);
end
