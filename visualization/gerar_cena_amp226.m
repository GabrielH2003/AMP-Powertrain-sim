function gerar_cena_amp226(arquivo, pista)
%GERAR_CENA_AMP226  Gera a cena VRML do kart AMP-226 para o Simulink 3D Animation.
%
%   gerar_cena_amp226('amp226.wrl')
%   gerar_cena_amp226('amp226.wrl', gerar_pista_fsae)   % com a pista de cones
%
%   Geometria com as dimensoes reais do carro. Os nos com DEF sao exatamente
%   os que o bloco VR Sink move durante o replay:
%
%     Carro     translation [x y z]       posicao no mundo
%     Carro     rotation    [0 1 0 psi]   guinada
%     Pitch     rotation    [0 0 1 th]    arfagem
%     Roll      rotation    [1 0 0 ph]    rolagem
%     Est_DE    rotation    [0 1 0 delta] estercamento (dianteiras)
%     Est_DD    rotation    [0 1 0 delta]
%     Gir_DE/DD/TE/TD  rotation [0 0 1 ang]  giro de cada roda
%
%   CONVENCAO DE EIXOS - a parte que mais da erro:
%     nosso modelo:  x=frente,  y=esquerda, z=cima
%     VRML:          x=direita, y=CIMA,     z=para o observador
%   Mapeamento usado (aplicado em prepara_replay3d.m):
%     vrml_x = +X_modelo,  vrml_y = altura,  vrml_z = -Y_modelo
%   Assim a guinada entra direto (psi_vr = +psi): uma rotacao positiva em
%   torno de +Y leva +X para -Z, que e justamente a nossa esquerda.
%   Rolagem e arfagem: ver prepara_replay3d.m (a rolagem troca de sinal
%   porque o eixo lateral inverteu).
%
%   Formato VRML97 (.wrl) e nao X3D: o parser do SL3D le VRML sem precisar
%   de DTD, o que evita centenas de avisos de validacao XML na abertura.

if nargin < 1, arquivo = 'amp226.wrl'; end
if nargin < 2
    if exist('pista_amp226.mat','file'), S = load('pista_amp226.mat'); pista = S.pista;
    else,                                pista = [];
    end
end
gerarTexturas();   % asfalto e grama procedurais, se ainda nao existirem

% --- dimensoes reais (AMP-226) ---
L   = 1.560;   a  = 0.804;   b = L - a;    % entre-eixos e posicao do CG
bit = 1.250;   Rw = 0.2605;  Wt = 0.183;   % bitola, raio de roda, largura do pneu
hcg = 0.295;                               % altura do CG (a origem do carro fica aqui)
Lch = 1.90;    Wch = 0.62;   Hch = 0.28;   % caixa do chassi

yr  = -hcg + Rw;                           % altura do centro das rodas, relativa ao CG
ych = -hcg + Rw + Hch/2;                   % altura do centro do chassi

s = {};
s{end+1} = '#VRML V2.0 utf8';
s{end+1} = '# Cena de replay do AMP-226 - Ampera Racing / UFSC';
s{end+1} = '# Gerada por gerar_cena_amp226.m - NAO editar a mao, regerar.';
s{end+1} = '';
s{end+1} = 'WorldInfo {';
s{end+1} = '  title "AMP-226"';
s{end+1} = '  info [ "Replay da simulacao de powertrain IRFOC" ]';
s{end+1} = '}';
% headlight FALSE de proposito: no canvas WebGL a luz presa na camera lava a
% cena inteira (grama fica verde-limao, asfalto fica claro). Sem ela quem
% ilumina sao as duas direcionais abaixo, que dao contraste e sombreado.
s{end+1} = 'NavigationInfo { type [ "EXAMINE" "ANY" ] headlight FALSE speed 8 }';
s{end+1} = 'Background {';
s{end+1} = '  skyColor [ 0.53 0.62 0.74, 0.85 0.89 0.94 ]';
s{end+1} = '  skyAngle [ 1.2 ]';
s{end+1} = '  groundColor [ 0.28 0.28 0.30 ]';
s{end+1} = '}';
s{end+1} = 'DirectionalLight { direction -0.4 -1 -0.3 intensity 1.0  ambientIntensity 0.35 }';
s{end+1} = 'DirectionalLight { direction  0.6 -0.5 0.5 intensity 0.30 ambientIntensity 0.15 }';
s{end+1} = '';

% ---- terreno ----
s{end+1} = '# ---------------- terreno ----------------';
s{end+1} = planoTexturado(600, 'grama.png', 200, -0.02, [0.42 0.55 0.30]);

% ---- pista ----
if ~isempty(pista)
    s = [s, pistaVRML(pista)];
else
    % sem pista: grade de 10 m, so para dar referencia de escala
    s{end+1} = '# grade de 10 m';
    for k = -10:10
      s{end+1} = caixa(k*10, 0.002, 0,   0.06, 0.004, 400, [0.42 0.44 0.46]);
      s{end+1} = caixa(0,    0.002, k*10, 400, 0.004, 0.06, [0.42 0.44 0.46]);
    end
    s{end+1} = caixa(0, 0.004, 0, 0.25, 0.006, 3.0, [0.95 0.95 0.95]);
end

s{end+1} = '';
% Camera de mapa. Fica FORA do no Carro de proposito: a translacao dela e
% dirigida pelos mesmos X e Y, mas ela nao herda a guinada. Assim o carro fica
% sempre no centro da tela e quem gira e a PISTA - que e o unico jeito de ver
% a trajetoria numa curva. As cameras dentro de Carro giram junto e a curva
% some.  Tela para cima = +X do modelo; tela para a direita = nossa direita.
s{end+1} = 'DEF CamMapa Transform {';
s{end+1} = '  translation 0 0 0';
s{end+1} = '  children [';
s{end+1} = '    DEF Aerea Viewpoint {';
s{end+1} = '      description "Aerea"';
s{end+1} = '      position 0 26 0';
s{end+1} = '      orientation -0.5774 -0.5774 -0.5774 2.0944';
s{end+1} = '    }';
s{end+1} = '  ]';
s{end+1} = '}';
s{end+1} = '';
s{end+1} = 'DEF VistaGeral Viewpoint {';
s{end+1} = '  description "Externa fixa"';
s{end+1} = '  position 6.500 3.200 7.000';
s{end+1} = '  orientation -0.3552 0.9243 0.1395 0.8035';
s{end+1} = '}';
s{end+1} = '';

% ================= O CARRO =================
s{end+1} = '# ---------------- o carro ----------------';
s{end+1} = 'DEF Carro Transform {';
s{end+1} = sprintf('  translation 0 %.4f 0', hcg);
s{end+1} = '  rotation 0 1 0 0';
s{end+1} = '  children [';
s{end+1} = '    DEF Perseguicao Viewpoint { description "Perseguicao" position -6.500 2.200 0.000  orientation -0.1403 -0.9801 -0.1403 1.5909 }';
s{end+1} = '    DEF Cockpit     Viewpoint { description "Cockpit"     position -0.150 0.550 0.000  orientation -0.0081 -0.9999 -0.0081 1.5709 }';
s{end+1} = '    DEF Lateral     Viewpoint { description "Lateral"     position 0.000 1.200 7.000  orientation -1.0000 0.0000 0.0000 0.1419 }';
s{end+1} = '    DEF Pitch Transform {';
s{end+1} = '      rotation 0 0 1 0';
s{end+1} = '      children [';
s{end+1} = '        DEF Roll Transform {';
s{end+1} = '          rotation 1 0 0 0';
s{end+1} = '          children [';

% chassi
s{end+1} = ind(12, caixa((a-b)/2, ych, 0, Lch, Hch, Wch, [0.09 0.17 0.34], 'chassi'));

% bico
s{end+1} = ind(12, sprintf(['Transform { translation %.4f %.4f 0  rotation 0 0 1 -1.5708 children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor 0.80 0.16 0.10 specularColor 0.3 0.3 0.3 } } ' ...
  'geometry Cone { bottomRadius %.4f height 0.55 } } ] }  # bico'], a+0.32, ych, Wch/2.4));

% arco de seguranca
s{end+1} = ind(12, sprintf(['Transform { translation %.4f %.4f 0  rotation 1 0 0 1.5708 children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor 0.75 0.76 0.78 } } ' ...
  'geometry Cylinder { radius 0.022 height %.4f } } ] }  # arco'], -b+0.30, ych + Hch/2 + 0.36, Wch*0.95));

% capacete do piloto
s{end+1} = ind(12, sprintf(['Transform { translation %.4f %.4f 0 children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor 0.93 0.93 0.90 } } ' ...
  'geometry Sphere { radius 0.13 } } ] }  # capacete'], -0.10, ych + Hch/2 + 0.20));

% asa traseira
s{end+1} = ind(12, caixa(-b-0.15, ych + 0.38, 0, 0.28, 0.02, Wch*1.5, [0.09 0.17 0.34], 'asa'));

% rodas: DE e DD tem estercamento; TE e TD nao
s{end+1} = ind(12, rodaVRML('DE',  a, yr, -bit/2, Rw, Wt, true));
s{end+1} = ind(12, rodaVRML('DD',  a, yr,  bit/2, Rw, Wt, true));
s{end+1} = ind(12, rodaVRML('TE', -b, yr, -bit/2, Rw, Wt, false));
s{end+1} = ind(12, rodaVRML('TD', -b, yr,  bit/2, Rw, Wt, false));

s{end+1} = '          ]';
s{end+1} = '        }';   % Roll
s{end+1} = '      ]';
s{end+1} = '    }';       % Pitch
s{end+1} = '  ]';
s{end+1} = '}';           % Carro

fid = fopen(arquivo,'w');
if fid < 0, error('nao consegui escrever em %s', arquivo); end
fprintf(fid, '%s\n', s{:});
fclose(fid);
fprintf('cena gerada: %s  (%d linhas)\n', arquivo, numel(s));
end

% =========================================================================
function s = pistaVRML(p)
%PISTAVRML  Fita de asfalto texturizada + os cones.
%   Os cones usam DEF/USE: a geometria e escrita UMA vez dentro de um Switch
%   invisivel e reaproveitada em cada posicao. Sem isso um traçado de 150
%   cones vira um .wrl enorme e o canvas engasga.

s = {};
s{end+1} = '# ---------------- pista ----------------';

% --- fita de asfalto ---
% Mais larga que a pista de cones: os cones ficam SOBRE o asfalto, com escape
% dos dois lados, como num autocross de verdade.
meia = p.largura/2 + 1.6;
n  = numel(p.s);
nx = -sin(p.psi);  ny = cos(p.psi);          % normal apontando para a esquerda
XL = p.x + meia.*nx;  YL = p.y + meia.*ny;
XR = p.x - meia.*nx;  YR = p.y - meia.*ny;

pts = cell(1,n); tex = cell(1,n);
for k = 1:n
    % VRML: x = X, y = altura, z = -Y  (a mesma conversao do resto da cena)
    pts{k} = sprintf('%.3f 0 %.3f, %.3f 0 %.3f', XL(k), -YL(k), XR(k), -YR(k));
    v = p.s(k)/4;                             % a textura repete a cada 4 m
    tex{k} = sprintf('0 %.3f, 1 %.3f', v, v);
end
idx = cell(1,n-1);
for k = 1:n-1
    a = 2*(k-1);                              % L(k) R(k) R(k+1) L(k+1)
    idx{k} = sprintf('%d %d %d %d -1', a, a+1, a+3, a+2);
end

s{end+1} = 'Shape {';
s{end+1} = '  appearance Appearance {';
s{end+1} = '    material Material { diffuseColor 1 1 1 specularColor 0.04 0.04 0.04 }';
s{end+1} = '    texture ImageTexture { url "asfalto.png" }';
s{end+1} = '  }';
s{end+1} = '  geometry IndexedFaceSet {';
s{end+1} = ['    coord Coordinate { point [ ' strjoin(pts, ', ') ' ] }'];
s{end+1} = ['    texCoord TextureCoordinate { point [ ' strjoin(tex, ', ') ' ] }'];
s{end+1} = ['    coordIndex [ ' strjoin(idx, ' ') ' ]'];
s{end+1} = ['    texCoordIndex [ ' strjoin(idx, ' ') ' ]'];
s{end+1} = '    solid FALSE  creaseAngle 3.14';
s{end+1} = '  }';
s{end+1} = '}';

% --- linha de largada, atravessada na pista ---
s{end+1} = sprintf(['Transform { translation %.3f 0.012 %.3f rotation 0 1 0 %.4f children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor 0.93 0.93 0.93 } } ' ...
  'geometry Box { size 0.30 0.004 %.3f } } ] }  # largada'], ...
  p.x(1), -p.y(1), p.psi(1), p.largura + 0.9);

% --- os cones ---
% ESCRITOS POR EXTENSO, um Transform completo por cone. A versao anterior usava
% DEF/USE dentro de um Switch{whichChoice -1}, que e o idioma VRML correto e
% deixa o arquivo bem menor - mas o canvas WebGL do R2025a NAO resolve essas
% referencias e simplesmente nao desenha cone nenhum. O renderizador offscreen
% desenhava, entao o bug so aparece em uma das duas telas. Como a cena precisa
% funcionar nas duas, paga-se o tamanho: ~140 cones x 2 formas ainda da um
% .wrl de poucas centenas de kB.
grupos = { p.cones_azul,           [0.05 0.25 0.80], p.cone_pequeno
           p.cones_amarelo,        [0.95 0.78 0.05], p.cone_pequeno
           p.cones_laranja,        [1.00 0.42 0.02], p.cone_pequeno
           p.cones_laranja_grande, [1.00 0.30 0.00], p.cone_grande  };
for g = 1:size(grupos,1)
    C = grupos{g,1};  cor = grupos{g,2};  cota = grupos{g,3};
    for k = 1:size(C,1)
        s{end+1} = coneInline(C(k,1), -C(k,2), cor, cota); %#ok<AGROW>
    end
end
end

% -------------------------------------------------------------------------
function c = coneInline(x, z, cor, cota)
%CONEINLINE  Um cone de pista em (x, z): cone colorido sobre uma base escura.
%   As cotas vem das regras FS (228x228x325 mm o pequeno, 285x285x505 o
%   grande), entao o tamanho na cena e o tamanho REAL - da para julgar
%   distancia contando cones na imagem.
r = cota.base/2;  h = cota.altura;
c = sprintf([ ...
 'Transform { translation %.3f 0 %.3f children [ ' ...
 'Transform { translation 0 %.4f 0 children [ Shape { appearance Appearance { ' ...
 'material Material { diffuseColor %.3f %.3f %.3f emissiveColor %.3f %.3f %.3f } } ' ...
 'geometry Cone { bottomRadius %.4f height %.4f } } ] } ' ...
 'Transform { translation 0 0.006 0 children [ Shape { appearance Appearance { ' ...
 'material Material { diffuseColor 0.10 0.10 0.10 } } ' ...
 'geometry Box { size %.4f 0.012 %.4f } } ] } ] }'], ...
 x, z, h/2, cor, cor*0.18, r, h, cota.base, cota.base);
end

% -------------------------------------------------------------------------
function t = planoTexturado(lado, textura, repet, alt, corFallback)
%PLANOTEXTURADO  Plano horizontal grande com textura repetida.
m = lado/2;
if exist(textura,'file')
  tex = sprintf('texture ImageTexture { url "%s" } textureTransform TextureTransform { scale %g %g }', textura, repet, repet);
  cor = '1 1 1';
else
  tex = '';
  cor = sprintf('%.3f %.3f %.3f', corFallback);
end
t = sprintf([ ...
 'Shape { appearance Appearance { material Material { diffuseColor %s } %s } ' ...
 'geometry IndexedFaceSet { coord Coordinate { point [ %g %g %g, %g %g %g, %g %g %g, %g %g %g ] } ' ...
 'texCoord TextureCoordinate { point [ 0 0, 1 0, 1 1, 0 1 ] } ' ...
 'coordIndex [ 0 1 2 3 -1 ] texCoordIndex [ 0 1 2 3 -1 ] solid FALSE } }'], ...
 cor, tex, -m,alt,-m, m,alt,-m, m,alt,m, -m,alt,m);
end

% -------------------------------------------------------------------------
function gerarTexturas()
%GERARTEXTURAS  Asfalto e grama procedurais, geradas na primeira vez.
%   Procedurais de proposito: a cena nao pode depender de um arquivo de
%   imagem que so existe nesta maquina.
if ~exist('asfalto.png','file')
    rng(7); A = 0.30 + 0.055*randn(256,256); A = imgaussfilt(A, 0.7);
    A = repmat(max(0,min(1,A)), 1, 1, 3);
    A(:,:,3) = min(1, A(:,:,3)*1.04);               % asfalto puxa de leve para o azul
    imwrite(A, 'asfalto.png');
end
if ~exist('grama.png','file')
    rng(11); G = imgaussfilt(randn(256,256), 1.1);
    G = (G - min(G(:))) / (max(G(:)) - min(G(:)));
    V = zeros(256,256,3);
    V(:,:,1) = 0.26 + 0.11*G;
    V(:,:,2) = 0.40 + 0.14*G;
    V(:,:,3) = 0.19 + 0.08*G;
    imwrite(V, 'grama.png');
end
end

% =========================================================================
function t = caixa(x, y, z, sx, sy, sz, cor, comentario)
%CAIXA  Uma Box posicionada, em uma linha so.
if nargin < 8, comentario = ''; end
t = sprintf(['Transform { translation %.4f %.4f %.4f children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor %.3f %.3f %.3f } } ' ...
  'geometry Box { size %.4f %.4f %.4f } } ] }'], x, y, z, cor(1), cor(2), cor(3), sx, sy, sz);
if ~isempty(comentario), t = [t '  # ' comentario]; end
end

% -------------------------------------------------------------------------
function t = ind(n, t)
%IND  Indenta um bloco (possivelmente multilinha) em n espacos.
pad = repmat(' ', 1, n);
t = [pad regexprep(t, '\n', ['\n' pad])];
end

% -------------------------------------------------------------------------
function c = rodaVRML(nome, x, y, z, Rw, Wt, temEst)
%RODAVRML  Hierarquia de uma roda.
%   O Cylinder do VRML tem eixo em Y; ele e deitado 90 graus em torno de X
%   para o eixo virar Z (a lateral do carro). O giro da roda entao acontece
%   em torno de Z, que passou a ser o proprio eixo do cubo.
%   Com estercamento:  Est_<nome> (em Y)  ->  Gir_<nome> (em Z)  -> geometria
%   Sem estercamento:  Gir_<nome> (em Z)  -> geometria

L = {};
if temEst
  L{end+1} = sprintf('DEF Est_%s Transform {', nome);
  L{end+1} = sprintf('  translation %.4f %.4f %.4f', x, y, z);
  L{end+1} =         '  rotation 0 1 0 0';
  L{end+1} =         '  children [';
  L{end+1} = sprintf('    DEF Gir_%s Transform {', nome);
  L{end+1} =         '      rotation 0 0 1 0';
  L{end+1} =         '      children [';
  base = 8;
else
  L{end+1} = sprintf('DEF Gir_%s Transform {', nome);
  L{end+1} = sprintf('  translation %.4f %.4f %.4f', x, y, z);
  L{end+1} =         '  rotation 0 0 1 0';
  L{end+1} =         '  children [';
  base = 4;
end

p = repmat(' ', 1, base);
% pneu
L{end+1} = sprintf(['%sTransform { rotation 1 0 0 1.5708 children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor 0.10 0.10 0.11 specularColor 0.2 0.2 0.2 } } ' ...
  'geometry Cylinder { radius %.4f height %.4f } } ] }'], p, Rw, Wt);
% marca radial branca: e o que deixa a rotacao da roda visivel
L{end+1} = sprintf(['%sTransform { translation 0 %.4f %.4f children [ ' ...
  'Shape { appearance Appearance { material Material { diffuseColor 0.88 0.88 0.88 } } ' ...
  'geometry Box { size 0.03 %.4f 0.004 } } ] }'], p, Rw*0.55, Wt/2+0.003, Rw*0.7);

if temEst
  L{end+1} = '      ]';
  L{end+1} = '    }';
end
L{end+1} = '  ]';
L{end+1} = '}';

c = strjoin(L, sprintf('\n'));
end
