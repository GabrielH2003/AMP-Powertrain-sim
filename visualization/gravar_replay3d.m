function gravar_replay3d(nomeVideo, camera, arquivoR)
%GRAVAR_REPLAY3D  Grava o video a partir de um R ja salvo, sem re-simular.
%
%   gravar_replay3d('volta.mp4')
%   gravar_replay3d('perseguicao.mp4','Perseguicao')
%
%   Util porque a simulacao completa leva dezenas de minutos: uma vez que
%   replay3d.mat existe, da para gerar quantos angulos de camera quiser de
%   graca. Escreve o andamento no mesmo replay3d.log de fazer_replay3d.

if nargin < 1 || isempty(nomeVideo), nomeVideo = 'volta.mp4';      end
if nargin < 2 || isempty(camera),    camera    = 'Externa fixa';   end
if nargin < 3 || isempty(arquivoR),  arquivoR  = 'replay3d.mat';   end

saida = pwd;   % troque para receber uma copia do video e do log em outra pasta

arqLog = fullfile(pwd,'replay3d.log');
log = @(varargin) reg(arqLog, saida, sprintf(varargin{:}));

try
    log('gravar_replay3d: carregando %s', arquivoR);
    S = load(arquivoR, 'R');  R = S.R;
    log('R tem %d quadros a %g Hz', numel(R.t), R.fps);

    log('gravando %s (camera %s) ...', nomeVideo, camera);
    ti = tic;
    animar_replay3d(R, struct('video', nomeVideo, 'camera', camera));
    log('video pronto em %.0f s', toc(ti));

    if ~strcmpi(fullfile(saida, nomeVideo), fullfile(pwd, nomeVideo))
        copyfile(nomeVideo, fullfile(saida, nomeVideo));
        log('copiado para a pasta de saida');
    end
    log('FIM OK');
catch ME
    log('FIM ERRO: %s', ME.message);
    for k = 1:numel(ME.stack)
        log('   em %s linha %d', ME.stack(k).name, ME.stack(k).line);
    end
end
end

function reg(arqLog, saida, msg)
linha = sprintf('%s  %s', datestr(now,'HH:MM:SS'), msg);
fid = fopen(arqLog,'a');
if fid > 0, fprintf(fid,'%s\n',linha); fclose(fid); end
try, copyfile(arqLog, fullfile(saida,'replay3d.log')); catch, end
disp(linha);
end
