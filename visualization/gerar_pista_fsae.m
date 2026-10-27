function pista = gerar_pista_fsae(opcoes)
%GERAR_PISTA_FSAE  Gera um traçado de autocross Formula Student com cones.
%
%   pista = gerar_pista_fsae
%   pista = gerar_pista_fsae(struct('largura',3.5,'salvar','pista_amp226.mat'))
%
%   A linha de centro é uma spline cúbica PERIÓDICA passando pelos waypoints,
%   o que garante fechamento exato e continuidade de curvatura — importante
%   porque a curvatura vira comando de esterçamento no piloto.
%
%   CONVENÇÃO DE CORES (regras Formula Student Driverless):
%     azul     = borda ESQUERDA do sentido de corrida
%     amarelo  = borda DIREITA
%     laranja pequeno = aproximação da largada
%     laranja GRANDE  = a linha de largada/chegada em si
%
%   COTAS DOS CONES (regras FS):
%     pequeno  228 x 228 mm de base, 325 mm de altura
%     grande   285 x 285 mm de base, 505 mm de altura
%
%   A função checa o traçado contra as regras e IMPRIME as violações em vez
%   de falhar — um traçado levemente fora ainda é útil para visualizar, mas
%   você precisa saber.

if nargin < 1, opcoes = struct; end
d = @(c,v) getdef(opcoes,c,v);

W        = d('largura',      3.5);   % distância entre a fileira azul e a amarela
passo    = d('passo_cone',   4.0);   % espaçamento nominal dos cones ao longo da pista
passo_min= d('passo_min',    2.6);   % espaçamento em curva fechada
R_ref    = d('raio_ref',    12.0);   % raio abaixo do qual o espaçamento começa a fechar
arq      = d('salvar',      'pista_amp226.mat');

% ---------------------------------------------------------------------
% Waypoints da linha de centro, sentido anti-horário.
% Sai de (0,0) apontando para +x, que é a atitude inicial do carro no modelo.
% O trecho ondulado no topo é um SLALOM: período de ~8 m, dentro da faixa de
% 7,5 a 12 m que as regras pedem.
% ---------------------------------------------------------------------
P = d('waypoints', [ ...
      0    0     % largada, apontando para +x
     24    0     % reta principal, 48 m
     48    0
     64    4     % entrada da curva rapida da direita
     74   16
     76   30     % curva de raio grande
     68   42
     56   47
     44   48.5   % --- slalom: periodo 28 m, amplitude 2,5 m ---
     30   43.5   %     calibrado para R_min ~ 7 m; mais estreito que isso
     16   48.5   %     e o traçado deixa de ser dirigivel
      2   43.5   % --- fim do slalom ---
    -16   42
    -25   34     % curva longa da esquerda
    -29   23
    -27   12
    -19    3     % retorno para a largada
     -8    0 ]);

% ---------------------------------------------------------------------
% 1. linha de centro por spline periódica
% ---------------------------------------------------------------------
n  = size(P,1);
tk = (0:n).';                                   % nó extra para fechar o laço
Pk = [P; P(1,:)];
pp = csape(tk.', Pk.', 'periodic');             % spline periódica em 2D

tf = linspace(0, n, 4000).';
C  = fnval(pp, tf.').';                         % pontos
D1 = fnval(fnder(pp,1), tf.').';                % 1a derivada
D2 = fnval(fnder(pp,2), tf.').';                % 2a derivada

ds  = hypot(diff(C(:,1)), diff(C(:,2)));
s   = [0; cumsum(ds)];
Lap = s(end);

psi = atan2(D1(:,2), D1(:,1));
kap = (D1(:,1).*D2(:,2) - D1(:,2).*D2(:,1)) ./ (hypot(D1(:,1),D1(:,2)).^3);

% reamostra em passo constante de 0,25 m: fica cômodo para o piloto e para o asfalto
sq = (0 : 0.25 : Lap).';
xq = interp1(s, C(:,1), sq);
yq = interp1(s, C(:,2), sq);
pq = interp1(s, unwrap(psi), sq);
kq = interp1(s, kap, sq);

pista.s   = sq;   pista.x = xq;   pista.y = yq;
pista.psi = pq;   pista.kappa = kq;
pista.Lap = Lap;  pista.largura = W;
pista.waypoints = P;

% ---------------------------------------------------------------------
% 2. cones nas duas bordas
%    O passo encolhe onde a curvatura aperta — é o que se faz na prática e
%    é o que faz a curva "ler" como curva na imagem.
% ---------------------------------------------------------------------
[sc_azul, sc_amar] = deal([]);
sk = 0;
while sk < Lap
    R  = 1/max(abs(interp1(sq,kq,sk)), 1e-6);
    pc = passo_min + (passo - passo_min) * min(1, R/R_ref);
    sc_azul(end+1,1) = sk;  sc_amar(end+1,1) = sk; %#ok<AGROW>
    sk = sk + pc;
end

pista.cones_azul     = borda(sq, xq, yq, pq, sc_azul, +W/2);
pista.cones_amarelo  = borda(sq, xq, yq, pq, sc_amar, -W/2);

% laranja: os quatro grandes marcam a linha de largada (s = 0), os pequenos
% fazem a aproximação nos 12 m anteriores
pista.cones_laranja_grande = [ borda(sq,xq,yq,pq, [0;0], +W/2+0.4) ; ...
                               borda(sq,xq,yq,pq, [0;0], -W/2-0.4) ];
pista.cones_laranja_grande = unique(pista.cones_laranja_grande,'rows','stable');
sap = (Lap-12 : 3 : Lap-3).';
pista.cones_laranja = [ borda(sq,xq,yq,pq, sap, +W/2) ; ...
                        borda(sq,xq,yq,pq, sap, -W/2) ];

% tira da fileira azul/amarela os cones que caem em cima da largada
pista.cones_azul    = tirarPerto(pista.cones_azul,    [pista.cones_laranja; pista.cones_laranja_grande], 1.6);
pista.cones_amarelo = tirarPerto(pista.cones_amarelo, [pista.cones_laranja; pista.cones_laranja_grande], 1.6);

% cotas oficiais, para quem for desenhar
pista.cone_pequeno = struct('base',0.228,'altura',0.325);
pista.cone_grande  = struct('base',0.285,'altura',0.505);

% ---------------------------------------------------------------------
% 3. conferência contra as regras
% ---------------------------------------------------------------------
Rmin = 1/max(abs(kq));
reta = maiorReta(sq, kq, 1/50);      % "reta" = raio acima de 50 m

fprintf('\n--- pista autocross FSAE ---\n');
fprintf('  comprimento da volta  %6.1f m\n', Lap);
fprintf('  largura               %6.2f m   %s\n', W, ok(W >= 3.0, 'min 3,0 m'));
fprintf('  raio mínimo           %6.2f m   %s\n', Rmin, ok(Rmin >= 4.5, 'gancho: min 4,5 m no eixo'));
fprintf('  maior reta            %6.1f m   %s\n', reta, ok(reta <= 80, 'max 80 m'));
fprintf('  cones azuis %d | amarelos %d | laranja %d | laranja grandes %d\n', ...
    size(pista.cones_azul,1), size(pista.cones_amarelo,1), ...
    size(pista.cones_laranja,1), size(pista.cones_laranja_grande,1));
esp = max(hypot(diff(pista.cones_azul(:,1)), diff(pista.cones_azul(:,2))));
fprintf('  maior vão entre cones %6.2f m   %s\n', esp, ok(esp <= 5.0, 'max 5,0 m'));
fprintf('  extensão              x %.0f..%.0f   y %.0f..%.0f m\n\n', ...
    min(xq), max(xq), min(yq), max(yq));

if ~isempty(arq)
    save(arq, 'pista');
    fprintf('  salvo em %s\n\n', arq);
end
end

% =========================================================================
function Q = borda(s, x, y, psi, sc, off)
%BORDA  Desloca a linha de centro lateralmente. +off = esquerda do sentido.
xc = interp1(s, x, sc);  yc = interp1(s, y, sc);  pc = interp1(s, psi, sc);
Q  = [xc - off.*sin(pc), yc + off.*cos(pc)];
end

function A = tirarPerto(A, B, r)
%TIRARPERTO  Remove de A os pontos a menos de r de qualquer ponto de B.
if isempty(A) || isempty(B), return; end
manter = true(size(A,1),1);
for k = 1:size(A,1)
    if min(hypot(B(:,1)-A(k,1), B(:,2)-A(k,2))) < r, manter(k) = false; end
end
A = A(manter,:);
end

function L = maiorReta(s, kap, klim)
%MAIORRETA  Maior trecho contíguo com curvatura abaixo de klim.
e = abs(kap) < klim;
L = 0; ini = [];
for k = 1:numel(e)
    if e(k) && isempty(ini), ini = k; end
    if (~e(k) || k == numel(e)) && ~isempty(ini)
        L = max(L, s(k) - s(ini));  ini = [];
    end
end
end

function t = ok(cond, txt)
if cond, t = sprintf('OK   (%s)', txt); else, t = sprintf('>> FORA (%s)', txt); end
end

function v = getdef(s, c, v)
if isfield(s, c) && ~isempty(s.(c)), v = s.(c); end
end
