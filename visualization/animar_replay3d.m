function animar_replay3d(R, opcoes)
%ANIMAR_REPLAY3D  Toca (ou grava em video) o replay 3D do AMP-226.
%
%   animar_replay3d(R)                          % janela ao vivo, tempo real
%   animar_replay3d(R, struct('video','volta.mp4'))
%   animar_replay3d(R, struct('camera','Perseguicao','vel',2))
%
%   R vem de prepara_replay3d. Opcoes (todas com padrao):
%     cena     'amp226.wrl'    arquivo da cena
%     camera   'Externa fixa'  'Perseguicao' | 'Cockpit' | 'Lateral' | 'Externa fixa'
%     vel      1               1 = tempo real, 2 = duas vezes mais rapido
%     video    ''              se preenchido, grava em vez de mostrar ao vivo
%     tam      [1000 620]      tamanho em pixels
%     hud      true            escreve tempo e velocidade no canto (so no video)
%
%   POR QUE NAO E UM BLOCO VR SINK: no R2025a o visualizador classico do
%   Simulink 3D Animation foi removido, entao o VR Sink escreve na cena mas
%   nao abre janela nenhuma. Aqui a cena e desenhada por vr.canvas (ao vivo)
%   ou vr.offscreenCanvas (video), que continuam funcionando. O modelo
%   pwt_replay3d.slx faz a mesma coisa pelo caminho do Simulink.

if nargin < 2, opcoes = struct; end
d = @(c,v) getfielddef(opcoes,c,v);
cena   = d('cena',   'amp226.wrl');
if ~isfile(cena) && ~isempty(which(cena)), cena = which(cena); end   % acha a cena pelo path
camera = d('camera', 'Externa fixa');
vel    = d('vel',    1);
video  = d('video',  '');
tam    = d('tam',    [1000 620]);
hud    = d('hud',    true);

gravando = ~isempty(video);

w = vrworld(cena);  open(w);
limpar = onCleanup(@() fecharTudo(w));

% os nos que serao movidos, na mesma ordem dos campos de R
alvos = { 'Carro','translation', 'CarroT'
          'Carro','rotation',    'CarroR'
          'Pitch','rotation',    'PitchR'
          'Roll' ,'rotation',    'RollR'
          'Est_DE','rotation',   'EstR'
          'Est_DD','rotation',   'EstR'
          'Gir_DE','rotation',   'GirD'
          'Gir_DD','rotation',   'GirD'
          'Gir_TE','rotation',   'GirT'
          'Gir_TD','rotation',   'GirT'
          'CamMapa','translation','CamT' };

% replay3d.mat gravado antes da camera de mapa existir nao tem R.CamT; da para
% deduzi-la da propria translacao do carro, que e o que ela segue.
if ~isfield(R,'CamT')
    R.CamT = [R.CarroT(:,1), zeros(size(R.CarroT,1),1), R.CarroT(:,3)];
end
nos = cellfun(@(n) vrnode(w,n), alvos(:,1), 'UniformOutput', false);

usaTela = false;
if gravando
    % Caminho preferido: renderizador offscreen. Ele so existe quando o MATLAB
    % esta em OpenGL nativo; se estiver em WebGL (veja rendererinfo), o contexto
    % nao pode ser criado e nenhum truque o cria - ai cai para a captura de tela.
    ufPrime = uifigure('Position',[10 10 320 220],'Name','(inicializando OpenGL)');
    cvPrime = vr.canvas(w, ufPrime, [0 0 320 220]);  %#ok<NASGU>
    drawnow; pause(1.5);
    try
        cv = vr.offscreenCanvas(w, tam);
        set(cv,'Viewpoint',camera);
        limparP = onCleanup(@() delete(ufPrime));  %#ok<NASGU>
    catch
        delete(ufPrime);
        fprintf(['  offscreen indisponivel (MATLAB em WebGL?) - gravando pela TELA.\n' ...
                 '  NAO cubra nem minimize a janela ate terminar.\n']);
        usaTela = true;
        info = prepararCapturaTela(w, tam, camera);
        cv   = info.cv;
        limparT = onCleanup(@() delete(info.uf));  %#ok<NASGU>
    end

    vw = VideoWriter(video, 'MPEG-4');
    vw.FrameRate = R.fps;  vw.Quality = 92;
    open(vw);
    limparV = onCleanup(@() close(vw));  %#ok<NASGU>
else
    uf = uifigure('Position',[60 60 tam],'Name','AMP-226 - replay');
    cv = vr.canvas(w, uf, [0 0 tam]);
    set(cv,'Viewpoint',camera);
end


N  = numel(R.t);
t0 = tic;
for k = 1:N
    for j = 1:size(alvos,1)
        setfield(nos{j}, alvos{j,2}, R.(alvos{j,3})(k,:));  %#ok<SFLD>
    end
    vrdrawnow;

    if gravando
        if usaTela
            drawnow;
            im = capturarTela(info);
        else
            im = capture(cv);
        end
        if hud
            v = 0;
            if k > 1
                dp = R.CarroT(k,[1 3]) - R.CarroT(k-1,[1 3]);
                v  = norm(dp) * R.fps * 3.6;
            end
            txt = sprintf(' t = %5.2f s     %5.1f km/h ', R.t(k), v);
            im  = insertText(im, [10 10], txt, 'FontSize', 22, ...
                             'BoxColor','black','BoxOpacity',0.55,'TextColor','white');
        end
        writeVideo(vw, im);
        if mod(k, R.fps*2) == 0
            fprintf('  gravando... %d/%d (%.0f%%)\n', k, N, 100*k/N);
        end
    else
        if ~isvalid(uf), break; end
        atraso = (R.t(k) - R.t(1))/vel - toc(t0);
        if atraso > 0, pause(atraso); else, drawnow limitrate; end
    end
end

if gravando
    fprintf('video pronto: %s  (%d quadros, %.1f s)\n', video, N, N/R.fps);
end
end

% =========================================================================
function info = prepararCapturaTela(w, tam, camera)
%PREPARARCAPTURATELA  Abre a janela ao vivo e prepara o recorte para o Robot.
%
%   Caminho ALTERNATIVO de gravacao, para quando vr.offscreenCanvas nao esta
%   disponivel - o que acontece sempre que o MATLAB esta renderizando por
%   WebGL em vez de OpenGL nativo (confira com rendererinfo). Nesse modo o
%   contexto offscreen simplesmente nao existe, e nenhum truque o cria.
%
%   Aqui a cena e desenhada na janela de verdade e capturada da TELA. Duas
%   consequencias praticas: a janela precisa ficar visivel e por cima durante
%   toda a gravacao, e nada pode passar na frente dela.

info.uf = uifigure('Position', [60 60 tam], 'Name', 'AMP-226 - gravando');
info.cv = vr.canvas(w, info.uf, [0 0 tam]);
drawnow; pause(2);
set(info.cv, 'Viewpoint', camera);
figure(info.uf); drawnow; pause(2);

% ESCALA DE DPI. O MATLAB reporta pixels LOGICOS; o java.awt.Robot captura
% pixels FISICOS. Num monitor a 125% (1536x864 logico sobre 1920x1080 fisico)
% ignorar isso desloca o recorte em dezenas de pixels e a captura pega a
% barra do MATLAB em vez da cena.
ss = get(0,'ScreenSize');
dm = java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment() ...
        .getDefaultScreenDevice().getDisplayMode();
esc = double(dm.getWidth()) / ss(3);

p = get(info.uf,'Position');
info.rect  = java.awt.Rectangle(round(p(1)*esc), round((ss(4)-p(2)-p(4))*esc), ...
                                round(p(3)*esc), round(p(4)*esc));
info.robot = java.awt.Robot();
info.esc   = esc;
end

% -------------------------------------------------------------------------
function im = capturarTela(info)
%CAPTURARTELA  Um quadro, direto da tela, ja recortado na janela.
bi = info.robot.createScreenCapture(info.rect);
W  = double(bi.getWidth);  H = double(bi.getHeight);
px = reshape(typecast(int32(bi.getRGB(0,0,W,H,[],0,W)),'uint8'), 4, W, H);
im = permute(px([3 2 1],:,:), [3 2 1]);      % BGRA -> RGB
end

% -------------------------------------------------------------------------
function v = getfielddef(s, c, v)
if isfield(s, c) && ~isempty(s.(c)), v = s.(c); end
end

function fecharTudo(w)
try, close(w); end %#ok<TRYNC>
try, delete(w); end %#ok<TRYNC>
end
