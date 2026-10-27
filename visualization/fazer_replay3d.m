function fazer_replay3d(nomeVideo, camera, tStop, pastaSaida)
%FAZER_REPLAY3D  Roda a simulacao, converte a pose e grava o video do replay.
%
%   fazer_replay3d                                  % volta.mp4, camera externa
%   fazer_replay3d('perseguicao.mp4','Perseguicao')
%   fazer_replay3d('curto.mp4','Externa fixa', 20)  % so os 20 primeiros s
%
%   Escreve o andamento em replay3d.log e salva R em replay3d.mat. O log e o
%   video sao copiados para PASTASAIDA (padrao: a pasta atual),
%   para que da para acompanhar de fora sem mandar comando nenhum ao MATLAB.
%
%   IMPORTANTE: enquanto isto roda, NAO mande outro comando ao MATLAB. Um
%   comando novo interrompe o sim() e a corrida morre com "Simulation aborted".

if nargin < 1 || isempty(nomeVideo), nomeVideo = 'volta.mp4';     end
if nargin < 2 || isempty(camera),    camera    = 'Externa fixa';  end
if nargin < 3,                       tStop     = [];              end
if nargin < 4 || isempty(pastaSaida)
    pastaSaida = pwd;   % passe outra pasta para receber uma copia do video e do log
end

arqLog = fullfile(pwd,'replay3d.log');
log = @(varargin) escrever(arqLog, pastaSaida, sprintf(varargin{:}));
delete_se(arqLog);
log('inicio  (pasta de saida: %s)', pastaSaida);

try
    mdl = 'pwtsimplificado';
    if ~bdIsLoaded(mdl), load_system(mdl); end

    args = {mdl};
    if ~isempty(tStop), args = [args, {'StopTime', num2str(tStop)}]; end

    log('simulando...');
    ti = tic;
    out = sim(args{:});
    log('simulacao terminou em %.0f s', toc(ti));

    R = prepara_replay3d(out, 60);
    save('replay3d.mat', 'R', '-v7.3');
    log('R salvo: %d quadros', numel(R.t));

    log('gravando video %s ...', nomeVideo);
    ti = tic;
    animar_replay3d(R, struct('video', nomeVideo, 'camera', camera));
    log('video pronto em %.0f s', toc(ti));

    if ~strcmpi(fullfile(pastaSaida, nomeVideo), fullfile(pwd, nomeVideo)), try
        copyfile(nomeVideo, fullfile(pastaSaida, nomeVideo));
        log('video copiado para a pasta de saida');
    catch, log('nao consegui copiar o video para a pasta de saida'); end, end

    log('FIM OK');
catch ME
    log('FIM ERRO: %s', ME.message);
    for k = 1:numel(ME.stack)
        log('   em %s linha %d', ME.stack(k).name, ME.stack(k).line);
    end
end
end

% -------------------------------------------------------------------------
function escrever(arqLog, pastaSaida, msg)
linha = sprintf('%s  %s', datestr(now,'HH:MM:SS'), msg);
fid = fopen(arqLog,'a');
if fid > 0, fprintf(fid, '%s\n', linha); fclose(fid); end
% espelha na pasta de saida para acompanhamento externo
try, copyfile(arqLog, fullfile(pastaSaida,'replay3d.log')); catch, end
disp(linha);
end

function delete_se(f)
if exist(f,'file'), delete(f); end
end
