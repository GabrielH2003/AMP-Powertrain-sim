function add_piloto(mdl, pista)
%ADD_PILOTO  Acrescenta ao modelo um piloto que segue a pista de cones.
%
%   add_piloto                       % usa pwtsimplificado e pista_amp226.mat
%
%   Cria o subsistema PILOTO no nivel topo, que publica dois Gotos:
%       delta_pp   esterçamento por PURE PURSUIT
%       ped_pp     acelerador por alvo de velocidade vindo da curvatura
%
%   TUDO E OPCIONAL, de proposito. Nada muda nos seus ensaios atuais:
%       steer_mode = 5   liga o pure pursuit (modos 1..4 seguem iguais)
%       piloto_on  = 1   liga o acelerador do piloto (0 = Signal Builder)
%   Os dois padroes ficam desligados.
%
%   POR QUE PURE PURSUIT E NAO UM PID DE ERRO LATERAL: o pure pursuit e
%   geometrico - ele resolve "que esterçamento leva o eixo traseiro ate aquele
%   ponto por um arco". Nao tem ganho para ajustar alem da distancia de mira,
%   nao acumula erro, e degrada de forma previsivel quando a pista aperta.
%   Para VALIDAR o powertrain, que e o objetivo aqui, isso basta e sobra; um
%   controlador melhor so mudaria o traçado no limite de aderencia.
%
%   FRENAGEM: o piloto so comanda acelerador de 0 a 1. A desaceleracao vem do
%   regen de lift-off, que ja esta no modelo. Se a pista exigir frenagem forte,
%   e aqui que entra o pedal de freio - ainda nao entrou.

if nargin < 1 || isempty(mdl),   mdl = 'pwtsimplificado';       end
if nargin < 2 || isempty(pista), S = load('pista_amp226.mat'); pista = S.pista; end

if ~bdIsLoaded(mdl), load_system(mdl); end
sub = [mdl '/Piloto'];

% ---------------------------------------------------------------------
% 1. parametros no Model Workspace
%    A pista vira tabela reamostrada em passo constante: o piloto anda por
%    INDICE, entao passo constante e o que torna "Ld metros a frente" um
%    simples deslocamento de indice, sem busca.
% ---------------------------------------------------------------------
passo = 0.5;
n_pts = numel(0 : passo : pista.Lap - passo);

% Os parametros vao para o CODIGO do Model Workspace, nao por assignin.
% ARMADILHA CARA: o Model Workspace deste modelo tem DataSource = "MATLAB Code".
% Qualquer variavel posta por assignin some no proximo Reinitialize, e o modelo
% so acusa muito depois, com "Invalid setting for parameter Value" num bloco
% qualquer - sem nenhuma pista de que a variavel deixou de existir.
% A pista e carregada de pista_amp226.mat por dentro do codigo, e nao gravada
% como literal, para o texto nao inchar com 530 pares de coordenadas.
mws = get_param(mdl,'ModelWorkspace');
if ~strcmp(mws.DataSource,'MATLAB Code')
    error('add_piloto:fonte', ...
      'esperava Model Workspace com DataSource "MATLAB Code", achei "%s"', mws.DataSource);
end

MARCA = '%% ==== PILOTO DE PISTA (rev. 11) ====';
FIM   = '%% ==== fim do piloto ====';

L = { MARCA
 '%  Acrescentado por add_piloto.m. Tudo aqui e OPCIONAL:'
 '%     steer_mode = 5  ->  estercamento por pure pursuit'
 '%     piloto_on  = 1  ->  acelerador pelo alvo de velocidade'
 '%  Com os dois em zero o modelo se comporta exatamente como antes.'
 'if exist(''pista_amp226.mat'',''file'')'
 '    pista_ = load(''pista_amp226.mat'');'
 '    pista_ = pista_.pista;'
 sprintf('    trk_passo = %g;', passo)
 '    s_    = transpose(0 : trk_passo : pista_.Lap - trk_passo);'
 '    trk_x = interp1(pista_.s, pista_.x,     s_);'
 '    trk_y = interp1(pista_.s, pista_.y,     s_);'
 '    trk_k = interp1(pista_.s, pista_.kappa, s_);'
 '    clear pista_ s_'
 'else'
 '    % Sem pista no disco o piloto nao tem para onde ir. Tabelas minimas so'
 '    % para o modelo continuar compilando com steer_mode ~= 5.'
 '    trk_passo = 0.5;  trk_x = [0;1];  trk_y = [0;0];  trk_k = [0;0];'
 'end'
 ''
 'pp_Ld_min  = 4.0;    % mira minima [m] - abaixo disso o carro serpenteia'
 'pp_Ld_k    = 0.55;   % a mira cresce com a velocidade: Ld = pp_Ld_k*Vx'
 'pp_v_max   = 22.0;   % teto de velocidade [m/s]'
 'pp_ay_max  = 9.0;    % aceleracao lateral admitida [m/s^2] ~ 0,9 g'
 'pp_kp_v    = 0.35;   % ganho do acelerador por erro de velocidade'
 'pp_kp_b    = 0.30;   % ganho do FREIO por excesso de velocidade [1/(m/s)]'
 'pp_v_banda = 0.8;    % banda morta antes de pisar no freio [m/s]. Abaixo dela'
 '                     % quem desacelera e o regen de lift-off, sem atrito.'
 'pp_frn_max = 0.90;   % pedal de freio maximo. E FRACAO 0..1, escala diferente'
 '                     % do acelerador - o Brake_pedal do modelo ja e normalizado.'
 'pp_prev    = 12.0;   % quantos metros a frente olhar a curvatura'
 'pp_Ts      = 0.01;   % taxa do piloto [s] - 100 Hz'
 'pp_ped_max = 100;    % fundo de escala do acelerador. NAO e 0..1: o "pedal"'
 '                     % deste modelo e requisicao de TORQUE (lookup [0:94]).'
 'piloto_on  = 0;      % 0 = quem manda no acelerador e o Signal Builder'
 'steer_mode = 1;      % 5 = pure pursuit'
 FIM
 '' };
bloco = strjoin(L, newline);

cod = mws.MATLABCode;
i0 = strfind(cod, MARCA);
if isempty(i0)
    cod = [cod newline newline bloco];
else
    % substitui o bloco anterior, para add_piloto poder rodar de novo sem duplicar
    i1 = strfind(cod, FIM);
    i1 = i1(i1 > i0(1));
    cod = [cod(1:i0(1)-1), bloco, cod(i1(1)+numel(FIM)+1 : end)];
end
mws.MATLABCode = cod;
reload(mws);

% ---------------------------------------------------------------------
% 2. o subsistema, bem abaixo de tudo que ja existe
% ---------------------------------------------------------------------
if getSimulinkBlockHandle(sub) > 0, delete_block(sub); end

y0 = 0;
bl = find_system(mdl,'SearchDepth',1,'Type','Block');
for k = 1:numel(bl)
    p = get_param(bl{k},'Position');
    y0 = max(y0, p(4));
end
y0 = y0 + 160;

add_block('built-in/Subsystem', sub, 'Position', [-460 y0 -240 y0+150]);
% esvazia o subsistema recem-criado (In1->Out1). O nome da linha varia entre
% releases, entao apaga por handle em vez de por nome.
lin = find_system(sub,'FindAll','on','Type','line');
if ~isempty(lin), delete_line(lin); end
dentro = find_system(sub,'SearchDepth',1,'Type','Block');
for k = 1:numel(dentro)
    if ~strcmp(dentro{k}, sub), delete_block(dentro{k}); end
end

fonte = { 'X_pos','F_X'; 'Y_pos','F_Y'; 'psi','F_psi'; 'Velocity','F_V' };
for k = 1:size(fonte,1)
    add_block('built-in/From', [sub '/' fonte{k,2}], ...
        'GotoTag', fonte{k,1}, 'Position', [40 40+60*k 110 68+60*k]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/PurePursuit'], ...
          'Position', [200 90 380 330]);


add_block('built-in/Goto', [sub '/G_delta'], 'GotoTag','delta_pp', ...
          'TagVisibility','global', 'Position', [460 120 540 148]);
add_block('built-in/Goto', [sub '/G_ped'],   'GotoTag','ped_pp', ...
          'TagVisibility','global', 'Position', [460 200 540 228]);
add_block('built-in/Goto', [sub '/G_frn'],   'GotoTag','frn_pp', ...
          'TagVisibility','global', 'Position', [460 260 540 288]);

% ---------------------------------------------------------------------
% 3. o algoritmo
% ---------------------------------------------------------------------
cod = [ ...
"function [delta, ped, frn, v_alvo] = fcn(X, Y, psi, Vx, trk_x, trk_y, trk_k, trk_passo, pp_Ld_min, pp_Ld_k, pp_v_max, pp_ay_max, pp_kp_v, pp_prev, L_wb, pp_ped_max, pp_kp_b, pp_v_banda, pp_frn_max)"
"%#codegen"
"% Piloto de pure pursuit. Roda em tempo discreto e guarda o indice do ultimo"
"% ponto mais proximo, o que permite buscar so numa JANELA a frente em vez de"
"% varrer a pista inteira a cada passo - com ~530 pontos e passo de 12,5 us a"
"% busca completa dominaria o custo da simulacao."
""
"persistent i0"
"if isempty(i0)"
"    i0 = int32(1);"
"end"
"N = int32(numel(trk_x));"
""
"% --- 1. ponto mais proximo, procurando so para a frente ---"
"% So para a frente de proposito: impede o piloto de 'voltar' e travar quando a"
"% pista passa perto dela mesma."
"melhor = inf;"
"ib = i0;"
"for j = int32(0):int32(60)"
"    id = mod(i0 - 1 + j, N) + 1;"
"    d2 = (trk_x(id) - X)^2 + (trk_y(id) - Y)^2;"
"    if d2 < melhor"
"        melhor = d2;"
"        ib = id;"
"    end"
"end"
"i0 = ib;"
""
"% --- 2. ponto de mira ---"
"% Ld cresce com a velocidade: perto do carro em baixa (precisao), longe em alta"
"% (estabilidade). Ld fixo da serpente em reta ou corta curva, dependendo do valor."
"Ld = pp_Ld_min;"
"if pp_Ld_k * Vx > Ld"
"    Ld = pp_Ld_k * Vx;"
"end"
"n  = int32(round(Ld / trk_passo));"
"if n < int32(1)"
"    n = int32(1);"
"end"
"it = mod(ib - 1 + n, N) + 1;"
""
"% --- 3. a geometria do pure pursuit ---"
"% alpha e o angulo entre o eixo do carro e a linha ate o ponto de mira. O arco"
"% que liga os dois tem raio Ld/(2 sin alpha); Ackermann converte raio em esterco."
"al = atan2(trk_y(it) - Y, trk_x(it) - X) - psi;"
"al = atan2(sin(al), cos(al));"
"delta = atan2(2 * L_ent * sin(al), Ld);"
""
"% --- 4. velocidade alvo pela curvatura que vem pela frente ---"
"% Olha pp_prev metros adiante e pega a curva mais fechada: v = sqrt(ay_max/k)."
"% E o mesmo criterio de elipse de atrito, so que na versao mais simples."
"kmax = 1e-4;"
"nf = int32(round(pp_prev / trk_passo));"
"for j = int32(0):nf"
"    id = mod(ib - 1 + j, N) + 1;"
"    ak = abs(trk_k(id));"
"    if ak > kmax"
"        kmax = ak;"
"    end"
"end"
"v_alvo = sqrt(pp_ay_max / kmax);"
"if v_alvo > pp_v_max"
"    v_alvo = pp_v_max;"
"end"
""
"% --- 5. acelerador e freio ---"
"% DUAS ESCALAS DIFERENTES, e confundi-las nao gera erro nenhum:"
"%   acelerador: o 'pedal' do modelo e requisicao de TORQUE (lookup [0:94]),"
"%               entao a fracao 0..1 e multiplicada por pp_ped_max."
"%   freio:      o Brake_pedal ja e FRACAO 0..1, vai direto."
"% Os dois nunca sao acionados juntos - e um piloto, nao um teste de banco."
"err = v_alvo - Vx;"
"ped = 0;"
"frn = 0;"
"if err >= 0"
"    frac = pp_kp_v * err;"
"    if frac > 1"
"        frac = 1;"
"    end"
"    ped = frac * pp_ped_max;"
"elseif err < -pp_v_banda"
"    % So pisa no freio quando o regen de lift-off nao da conta. A banda morta"
"    % evita alternar acelerador/freio a cada passo em torno da velocidade alvo,"
"    % que e o que faz um piloto ruim (e um controlador sem histerese)."
"    frn = pp_kp_b * (-err - pp_v_banda);"
"    if frn > pp_frn_max"
"        frn = pp_frn_max;"
"    end"
"end"
"end"];
cod = strrep(strjoin(cod, newline), 'L_ent', 'L_wb');
% L_wb entra pela assinatura como os demais: variavel usada e nao declarada
% vira erro de dimensao no bloco, e a mensagem que o Simulink da nao aponta
% para ela.

rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path',[sub '/PurePursuit']);
ch.Script = cod;

% Taxa DISCRETA explicita. Sem isso o bloco herda tempo continuo e o Simulink
% recusa a variavel persistent - a mensagem fala em "constructs invalidos" e
% nao deixa nada obvio que o problema e o sample time. 100 Hz tambem e a taxa
% em que um piloto, humano ou autonomo, de fato atualiza o volante.
% Vai pelo objeto do chart: o bloco MATLAB Function e um SubSystem mascarado
% e NAO tem parametro 'SampleTime' para set_param.
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime  = 'pp_Ts';

% re-declara os parametros: reescrever o Script apaga o escopo Parameter
nomes = {'trk_x','trk_y','trk_k','trk_passo','pp_Ld_min','pp_Ld_k', ...
         'pp_v_max','pp_ay_max','pp_kp_v','pp_prev','L_wb','pp_ped_max', ...
         'pp_kp_b','pp_v_banda','pp_frn_max'};
ch = rt.find('-isa','Stateflow.EMChart','Path',[sub '/PurePursuit']);
for k = 1:numel(nomes)
    d = ch.find('-isa','Stateflow.Data','-and','Name',nomes{k});
    if ~isempty(d), d(1).Scope = 'Parameter'; end
end

% ---------------------------------------------------------------------
% 4. fiacao interna
% ---------------------------------------------------------------------
for k = 1:4
    add_line(sub, sprintf('%s/1', fonte{k,2}), sprintf('PurePursuit/%d', k), 'autorouting','on');
end
add_line(sub, 'PurePursuit/1', 'G_delta/1', 'autorouting','on');
add_line(sub, 'PurePursuit/2', 'G_ped/1',   'autorouting','on');
add_line(sub, 'PurePursuit/3', 'G_frn/1',   'autorouting','on');

add_block('built-in/Terminator', [sub '/T1'], 'Position',[460 320 480 340]);
add_line(sub, 'PurePursuit/4', 'T1/1', 'autorouting','on');   % v_alvo, so p/ log

Simulink.BlockDiagram.arrangeSystem(sub);   % so no subsistema NOVO

% ---------------------------------------------------------------------
% 5. modo 5 na Direcao — puramente aditivo
% ---------------------------------------------------------------------
dir = [mdl '/Direcao'];
sel = [dir '/Sel_modo'];
if str2double(get_param(sel,'Inputs')) < 5
    ymax = 0;
    bl = find_system(dir,'SearchDepth',1,'Type','Block');
    for k = 1:numel(bl)
        if strcmp(bl{k}, dir), continue; end
        p = get_param(bl{k},'Position');  ymax = max(ymax, p(4));
    end
    if getSimulinkBlockHandle([dir '/F_delta_pp']) <= 0
        add_block('built-in/From', [dir '/F_delta_pp'], 'GotoTag','delta_pp', ...
                  'Position', [40 ymax+50 140 ymax+78]);
    end
    set_param(sel, 'Inputs', '5');
    add_line(dir, 'F_delta_pp/1', 'Sel_modo/6', 'autorouting','on');
end
% NAO roda arrangeSystem na Direcao: o usuario arrumou esse subsistema a mao.


% ---------------------------------------------------------------------
% 6. acelerador: um Switch logo depois do Signal Builder
%    Esta e a UNICA alteracao na fiacao que ja existia. Feita por HANDLE de
%    porta, nao por nome: o bloco 'Lookup Table' tem quebra de linha no nome e
%    add_line/delete_line por nome falham nele.
%    Com piloto_on = 0 o caminho e exatamente o de antes.
% ---------------------------------------------------------------------
sb  = [mdl '/Signal Builder1'];
swb = [mdl '/Sw_ped'];
if getSimulinkBlockHandle(swb) <= 0
    phs = get_param(sb, 'PortHandles');
    lin = get_param(phs.Outport(1), 'Line');
    dst = get_param(lin, 'DstPortHandle');      % lookup de torque + Goto_ped
    delete_line(lin);

    pos = get_param(sb, 'Position');
    yb  = pos(4) + 70;
    add_block('built-in/Switch', swb, ...
        'Position', [pos(1)+20 yb pos(1)+60 yb+70], ...
        'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
    add_block('built-in/From', [mdl '/F_ped_pp'], 'GotoTag','ped_pp', ...
        'Position', [pos(1)-130 yb-30 pos(1)-30 yb-2]);
    add_block('built-in/Constant', [mdl '/C_piloto_on'], 'Value','piloto_on', ...
        'Position', [pos(1)-130 yb+22 pos(1)-30 yb+50]);

    phw = get_param(swb,'PortHandles');
    add_line(mdl, get_param([mdl '/F_ped_pp'],  'PortHandles').Outport(1), phw.Inport(1), 'autorouting','on');
    add_line(mdl, get_param([mdl '/C_piloto_on'],'PortHandles').Outport(1), phw.Inport(2), 'autorouting','on');
    add_line(mdl, phs.Outport(1), phw.Inport(3), 'autorouting','on');
    for k = 1:numel(dst)
        add_line(mdl, phw.Outport(1), dst(k), 'autorouting','on');
    end
end
% NAO roda arrangeSystem no nivel topo: o usuario arrumou esse diagrama a mao.


% ---------------------------------------------------------------------
% 7. freio: um Switch antes do Goto_brk, dentro do Subsystem1
%    Mesma logica do acelerador, mesma chave piloto_on. O Sum_brk alimenta
%    SO o Goto_brk, entao a insercao e limpa - uma linha entra, uma sai.
% ---------------------------------------------------------------------
s1  = [mdl '/Subsystem1'];
swf = [s1 '/Sw_frn'];
if getSimulinkBlockHandle(swf) <= 0
    phs = get_param([s1 '/Sum_brk'], 'PortHandles');
    lin = get_param(phs.Outport(1), 'Line');
    dst = get_param(lin, 'DstPortHandle');
    delete_line(lin);

    ymax = 0;
    bl = find_system(s1,'SearchDepth',1,'Type','Block');
    for k = 1:numel(bl)
        if strcmp(bl{k}, s1), continue; end
        q = get_param(bl{k},'Position');  ymax = max(ymax, q(4));
    end
    yb = ymax + 60;

    add_block('built-in/Switch', swf, 'Position', [190 yb 230 yb+70], ...
              'Criteria','u2 >= Threshold', 'Threshold','0.5');
    add_block('built-in/From', [s1 '/F_frn_pp'], 'GotoTag','frn_pp', ...
              'Position', [30 yb-30 130 yb-2]);
    add_block('built-in/Constant', [s1 '/C_piloto_on2'], 'Value','piloto_on', ...
              'Position', [30 yb+22 130 yb+50]);

    phw = get_param(swf,'PortHandles');
    add_line(s1, get_param([s1 '/F_frn_pp'],     'PortHandles').Outport(1), phw.Inport(1), 'autorouting','on');
    add_line(s1, get_param([s1 '/C_piloto_on2'],'PortHandles').Outport(1), phw.Inport(2), 'autorouting','on');
    add_line(s1, phs.Outport(1), phw.Inport(3), 'autorouting','on');
    for k = 1:numel(dst)
        add_line(s1, phw.Outport(1), dst(k), 'autorouting','on');
    end
end

fprintf(['\npiloto instalado.\n' ...
         '  pista: %.0f m, %d pontos a cada %.2f m\n' ...
         '  para usar:  steer_mode = 5   (esterçamento)\n' ...
         '              piloto_on  = 1   (acelerador E freio)\n' ...
         '  ambos comecam desligados - seus ensaios atuais nao mudam.\n\n'], ...
    pista.Lap, n_pts, passo);
end
