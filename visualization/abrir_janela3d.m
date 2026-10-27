function abrir_janela3d(atraso, camera)
%ABRIR_JANELA3D  Abre uma janela 3D sobre o mundo que o bloco VR Sink esta usando.
%
%   No R2025a o visualizador classico do Simulink 3D Animation foi removido:
%   o VR Sink continua ESCREVENDO na cena, mas nao abre janela nenhuma (a
%   opcao "Open Viewer automatically" nao faz mais nada). Esta funcao acha o
%   mundo que o bloco abriu e pendura nele um vr.canvas dentro de um uifigure,
%   que e o caminho que ainda funciona.
%
%   Chamada pelo StartFcn de pwt_replay3d.slx. O atraso existe porque o mundo
%   so aparece depois que o solver inicializa o bloco.

if nargin < 1 || isempty(atraso), atraso = 1.5;            end
if nargin < 2 || isempty(camera), camera = 'Externa fixa'; end

t = timer('StartDelay', atraso, 'ExecutionMode','singleShot', ...
          'Name','janela3d', 'TimerFcn', @(o,~) pendurar(o, camera));
start(t);
end

% -------------------------------------------------------------------------
function pendurar(o, camera)
try
    ws = vrwho;
    if isempty(ws)
        warning('abrir_janela3d:semMundo', ...
                'nenhum mundo VRML aberto - o VR Sink ja inicializou?');
        return
    end
    w = ws(end);

    % reaproveita a janela anterior se ainda estiver de pe
    uf = findall(0,'Type','figure','Tag','janela3d_amp226');
    if isempty(uf)
        uf = uifigure('Position',[60 60 1000 620], ...
                      'Name','AMP-226 - VR Sink', 'Tag','janela3d_amp226');
    else
        uf = uf(1);
    end
    delete(findall(uf,'Type','uicontrol'));

    c = vr.canvas(w, uf, [0 0 1000 620]);
    try, set(c,'Viewpoint',camera); catch, end
    figure(uf);
catch ME
    warning('abrir_janela3d:falhou','%s', ME.message);
end
try, stop(o); delete(o); catch, end
end
