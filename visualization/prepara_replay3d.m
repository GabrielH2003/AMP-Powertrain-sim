function R = prepara_replay3d(fonte, fps, hcg)
%PREPARA_REPLAY3D  Converte o sinal 'pose3d' da simulacao para as convencoes VRML.
%
%   R = prepara_replay3d(out)            % Simulink.SimulationOutput
%   R = prepara_replay3d(logsout)        % Simulink.SimulationData.Dataset
%   R = prepara_replay3d('matlab.mat')   % arquivo salvo
%   R = prepara_replay3d(M)              % matriz [t, 9] ou timeseries
%   R = prepara_replay3d(..., 60)        % taxa de quadros (padrao 60 Hz)
%
%   O sinal de entrada e o Goto global 'pose3d', montado no subsistema Pose 3D:
%
%     1 X_pos   [m]    2 Y_pos [m]      3 Z (=0)      [m]
%     4 roll    [rad]  5 pitch [rad]    6 psi         [rad]
%     7 delta   [rad]  8 th_roda_tras   9 th_roda_diant [rad]
%
%   ---------------------------------------------------------------------
%   AS DUAS CONVERSOES QUE IMPORTAM
%   ---------------------------------------------------------------------
%   (1) EIXOS. Nosso modelo usa x=frente, y=esquerda, z=cima. O VRML usa
%       x=direita, y=CIMA, z=para o observador. O mapeamento e:
%
%           vrml_x = +X_modelo      vrml_y = altura      vrml_z = -Y_modelo
%
%       Com isso a guinada entra SEM troca de sinal: uma rotacao positiva em
%       torno de +Y leva +X para -Z, que e exatamente a nossa esquerda.
%       Verificado na cena com psi=+45deg -> nariz vai para a esquerda.
%
%   (2) SINAIS DE ROLL E PITCH. O bloco 'Angulos de Suspensao' define
%
%           roll  = -deg2rad(roll_grad) *(ay/g)
%           pitch = -deg2rad(pitch_grad)*(ax/g)
%
%       ou seja, NESTE modelo curva a esquerda (ay>0) da roll NEGATIVO e
%       aceleracao (ax>0) da pitch NEGATIVO. Na cena, rotacao positiva em +X
%       baixa o lado direito e rotacao positiva em +Z levanta o nariz - que e
%       justamente o que a fisica pede nesses dois casos. Logo AMBOS invertem:
%
%           roll_vr = -roll        pitch_vr = -pitch
%
%   (3) GIRO DAS RODAS. Rotacao positiva em +Z leva +X (frente) para +Y
%       (cima), isto e, o topo da roda anda para TRAS. Andando para a frente
%       o topo tem de ir para a frente, entao tambem inverte:
%
%           ang_vr = -th_roda
%
%   ---------------------------------------------------------------------
%   R traz os campos ja prontos para o VR Sink e a matriz R.M (N x 39) com
%   tudo concatenado, para alimentar um unico bloco From Workspace.

if nargin < 2 || isempty(fps), fps = 60;    end
if nargin < 3 || isempty(hcg), hcg = 0.295; end   % TEM de bater com gerar_cena_amp226.m

% ---------- 1. extrair [t, 9] de qualquer formato de entrada ----------
[t, P] = extrair(fonte);
if size(P,2) ~= 9
    error('prepara_replay3d:largura', ...
          'esperava 9 colunas em pose3d, recebi %d. O Mux do subsistema Pose 3D mudou?', size(P,2));
end

% ---------- 2. reamostrar para a taxa de quadros ----------
[t, iu] = unique(t(:), 'stable');  P = P(iu,:);
tq = (t(1) : 1/fps : t(end)).';
Q  = interp1(t, P, tq, 'linear');

N = numel(tq);
X = Q(:,1); Y = Q(:,2);
roll = Q(:,4); pitch = Q(:,5); psi = Q(:,6);
delta = Q(:,7); thT = Q(:,8); thD = Q(:,9);

% ---------- 3. aplicar as convencoes ----------
z1 = zeros(N,1); u1 = ones(N,1);

R.t      = tq;
R.fps    = fps;
R.CarroT = [ X,  hcg + Q(:,3),  -Y ];          % translacao no mundo
R.CarroR = [ z1, u1, z1,  psi   ];             % guinada  (sem inversao)
R.PitchR = [ z1, z1, u1, -pitch ];             % arfagem  (invertida)
R.RollR  = [ u1, z1, z1, -roll  ];             % rolagem  (invertida)
R.EstR   = [ z1, u1, z1,  delta ];             % estercamento (sem inversao)
R.GirD   = [ z1, z1, u1, -thD   ];             % rodas dianteiras (invertida)
R.GirT   = [ z1, z1, u1, -thT   ];             % rodas traseiras  (invertida)
R.CamT   = [ X,  z1,          -Y ];            % camera de mapa: segue a posicao,
                                               % nao a guinada (ver gerar_cena)

% ---------- 4. matriz unica para o From Workspace ----------
% ordem das colunas: ver R.mapa
R.M = [R.CarroT, R.CarroR, R.PitchR, R.RollR, R.EstR, R.EstR, ...
       R.GirD, R.GirD, R.GirT, R.GirT, R.CamT];
R.mapa = { 'Carro.translation', 1:3
           'Carro.rotation',    4:7
           'Pitch.rotation',    8:11
           'Roll.rotation',    12:15
           'Est_DE.rotation',  16:19
           'Est_DD.rotation',  20:23
           'Gir_DE.rotation',  24:27
           'Gir_DD.rotation',  28:31
           'Gir_TE.rotation',  32:35
           'Gir_TD.rotation',  36:39
           'CamMapa.translation', 40:42 };

% para o bloco From Workspace: variavel [t, dados]
R.FromWorkspace = [tq, R.M];

fprintf(['prepara_replay3d: %d amostras -> %d quadros a %g Hz (%.2f s)\n' ...
         '  percurso  X %.1f..%.1f m   Y %.1f..%.1f m\n' ...
         '  psi %.1f..%.1f deg | roll max %.2f deg | pitch %.2f..%.2f deg\n' ...
         '  volta das rodas: traseira %.0f  dianteira %.0f\n'], ...
    numel(t), N, fps, tq(end)-tq(1), min(X), max(X), min(Y), max(Y), ...
    rad2deg(min(psi)), rad2deg(max(psi)), rad2deg(max(abs(roll))), ...
    rad2deg(min(pitch)), rad2deg(max(pitch)), thT(end)/(2*pi), thD(end)/(2*pi));
end

% =========================================================================
function [t, P] = extrair(fonte)
%EXTRAIR  Aceita SimulationOutput, Dataset, timeseries, matriz ou nome de .mat.

if ischar(fonte) || isstring(fonte)
    S = load(char(fonte));
    f = fieldnames(S);
    alvo = [];
    for k = 1:numel(f)
        if strcmpi(f{k},'pose3d'), alvo = S.(f{k}); break; end
    end
    if isempty(alvo)
        for k = 1:numel(f)
            v = S.(f{k});
            if isa(v,'Simulink.SimulationOutput') || isa(v,'Simulink.SimulationData.Dataset')
                alvo = v; break;
            end
        end
    end
    if isempty(alvo), error('nao achei pose3d nem logsout em %s', char(fonte)); end
    [t, P] = extrair(alvo);
    return
end

if isa(fonte,'Simulink.SimulationOutput')
    [t, P] = extrair(fonte.logsout);  return
end

if isa(fonte,'Simulink.SimulationData.Dataset')
    e = fonte.getElement('pose3d');
    [t, P] = extrair(e.Values);  return
end

if isa(fonte,'timeseries')
    t = fonte.Time(:);
    P = squeeze(fonte.Data);
    if size(P,1) ~= numel(t), P = P.'; end
    return
end

if isstruct(fonte) && isfield(fonte,'time') && isfield(fonte,'signals')
    t = fonte.time(:);  P = fonte.signals.values;  return
end

if isnumeric(fonte)
    if size(fonte,2) == 10          % [t, 9]
        t = fonte(:,1);  P = fonte(:,2:end);
    else
        error('matriz numerica precisa ser [t, 9] (10 colunas)');
    end
    return
end

error('nao sei ler uma fonte do tipo %s', class(fonte));
end
