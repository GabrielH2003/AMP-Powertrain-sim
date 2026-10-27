function dump_modelo(mdl, arqSaida)
%DUMP_MODELO  Escreve um relatorio textual da estrutura de um modelo Simulink.
%   Para cada subsistema: blocos com seus parametros relevantes e a fiacao
%   (origem -> destino de cada linha), alem de Froms/Gotos.

if nargin < 1, mdl = 'pwtsimplificado'; end
if nargin < 2, arqSaida = 'estrutura_modelo.txt'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

fid = fopen(arqSaida,'w','n','UTF-8');
p = @(varargin) fprintf(fid, varargin{:});

alvos = { mdl
          [mdl '/Direcao']
          [mdl '/Piloto']
          [mdl '/Regen']
          [mdl '/Monitor DC']
          [mdl '/Pose 3D']
          [mdl '/Pose 3D/Angulos de Suspensao']
          [mdl '/Veículo']
          [mdl '/Transmissão']
          [mdl '/Pneu']
          [mdl '/Pneu Dianteiro']
          [mdl '/Subsystem']
          [mdl '/Subsystem1'] };

for a = 1:numel(alvos)
    sys = alvos{a};
    if getSimulinkBlockHandle(sys) <= 0 && ~strcmp(sys,mdl)
        p('\n===== %s : NAO EXISTE =====\n', sys); continue
    end
    p('\n\n========================================================\n');
    p('SISTEMA: %s\n', sys);
    p('========================================================\n');

    % --- portas do subsistema
    if ~strcmp(sys, mdl)
        ins  = find_system(sys,'SearchDepth',1,'BlockType','Inport');
        outs = find_system(sys,'SearchDepth',1,'BlockType','Outport');
        if ~isempty(ins)
            p('ENTRADAS: ');
            for k=1:numel(ins), p('%d=%s  ', str2double(get_param(ins{k},'Port')), get_param(ins{k},'Name')); end
            p('\n');
        end
        if ~isempty(outs)
            p('SAIDAS  : ');
            for k=1:numel(outs), p('%d=%s  ', str2double(get_param(outs{k},'Port')), get_param(outs{k},'Name')); end
            p('\n');
        end
    end

    bl = find_system(sys,'SearchDepth',1,'Type','Block');
    p('\n-- BLOCOS (%d) --\n', numel(bl));
    for k = 1:numel(bl)
        b = bl{k};
        if strcmp(b, sys), continue; end
        nome = strrep(get_param(b,'Name'), newline, ' ');
        tipo = get_param(b,'BlockType');
        det = '';
        try
            switch tipo
                case 'Gain',        det = sprintf('  K = %s', get_param(b,'Gain'));
                case 'Sum',         det = sprintf('  sinais = %s', get_param(b,'Inputs'));
                case 'Constant',    det = sprintf('  valor = %s', get_param(b,'Value'));
                case 'Saturate',    det = sprintf('  [%s , %s]', get_param(b,'LowerLimit'), get_param(b,'UpperLimit'));
                case 'Integrator',  det = sprintf('  IC = %s | limites %s [%s , %s]', get_param(b,'InitialCondition'), get_param(b,'LimitOutput'), get_param(b,'LowerSaturationLimit'), get_param(b,'UpperSaturationLimit'));
                case 'Switch',      det = sprintf('  criterio: u2 %s %s', get_param(b,'Criteria'), get_param(b,'Threshold'));
                case 'MultiPortSwitch', det = sprintf('  n entradas = %s', get_param(b,'Inputs'));
                case 'Product',     det = sprintf('  %s | entradas = %s', get_param(b,'Multiplication'), get_param(b,'Inputs'));
                case 'Goto',        det = sprintf('  tag = %s (%s)', get_param(b,'GotoTag'), get_param(b,'TagVisibility'));
                case 'From',        det = sprintf('  tag = %s', get_param(b,'GotoTag'));
                case 'RateLimiter', det = sprintf('  subida %s | descida %s', get_param(b,'RisingSlewLimit'), get_param(b,'FallingSlewLimit'));
                case 'Step',        det = sprintf('  t = %s | %s -> %s', get_param(b,'Time'), get_param(b,'Before'), get_param(b,'After'));
                case 'Lookup_n-D',  det = sprintf('  bp = %s | tabela = %s', get_param(b,'BreakpointsForDimension1'), get_param(b,'Table'));
                case 'Trigonometry',det = sprintf('  %s', get_param(b,'Operator'));
                case 'Math',        det = sprintf('  %s', get_param(b,'Operator'));
                case 'MinMax',      det = sprintf('  %s | entradas = %s', get_param(b,'Function'), get_param(b,'Inputs'));
                case 'Abs',         det = '';
                case 'UnitDelay',   det = sprintf('  IC = %s | Ts = %s', get_param(b,'InitialCondition'), get_param(b,'SampleTime'));
                case 'Demux',       det = sprintf('  saidas = %s', get_param(b,'Outputs'));
                case 'Mux',         det = sprintf('  entradas = %s', get_param(b,'Inputs'));
                case 'Selector',    det = sprintf('  indices = %s', get_param(b,'IndexParamArray'));
                case 'DiscreteIntegrator', det = sprintf('  IC = %s | K = %s | limites %s [%s , %s]', get_param(b,'InitialCondition'), get_param(b,'gainval'), get_param(b,'LimitOutput'), get_param(b,'LowerSaturationLimit'), get_param(b,'UpperSaturationLimit'));
                case 'ToWorkspace', det = sprintf('  var = %s', get_param(b,'VariableName'));
                case 'TransferFcn', det = sprintf('  num = %s | den = %s', get_param(b,'Numerator'), get_param(b,'Denominator'));
                case 'DiscreteTransferFcn', det = sprintf('  num = %s | den = %s', get_param(b,'Numerator'), get_param(b,'Denominator'));
                case 'Fcn',         det = sprintf('  expr = %s', get_param(b,'Expr'));
            end
        catch
        end
        p('  %-34s %-22s%s\n', nome, tipo, det);
    end

    % --- codigo dos blocos MATLAB Function deste nivel
    subs = find_system(sys,'SearchDepth',1,'BlockType','SubSystem');
    rt = sfroot;
    for k = 1:numel(subs)
        try
            ch = rt.find('-isa','Stateflow.EMChart','Path',subs{k});
            if ~isempty(ch)
                p('\n-- CODIGO de %s --\n', get_param(subs{k},'Name'));
                p('%s\n', ch(1).Script);
            end
        catch
        end
    end

    % --- fiacao
    lns = find_system(sys,'SearchDepth',1,'FindAll','on','Type','line');
    p('\n-- LIGACOES --\n');
    for k = 1:numel(lns)
        s = get_param(lns(k),'SrcBlockHandle');
        if s <= 0, continue; end
        sp = get_param(lns(k),'SrcPortHandle');
        np = ''; try, np = num2str(get_param(sp(1),'PortNumber')); catch, end
        nm = get_param(lns(k),'Name');
        if isempty(nm), nm = '-'; end
        nomeS = strrep(char(get_param(s,'Name')), newline, ' ');

        % Uma linha pode ter varios destinos (ramificacao). Trata sempre como
        % vetor: get_param num vetor de handles devolve CELL, e o fprintf
        % engasga - foi exatamente o erro que este bloco corrige.
        ds = get_param(lns(k),'DstPortHandle');
        ds = ds(ds > 0);
        if isempty(ds)
            p('  %-30s [%s] --%-14s--> (sem destino)\n', nomeS, np, nm);
            continue
        end
        for j = 1:numel(ds)
            bd = get_param(ds(j),'Parent');
            nomeD = strrep(char(bd), newline, ' ');
            nomeD = strrep(nomeD, [sys '/'], '');
            dn = ''; try, dn = num2str(get_param(ds(j),'PortNumber')); catch, end
            p('  %-30s [%s] --%-14s--> %-30s [%s]\n', nomeS, np, nm, nomeD, dn);
        end
    end
end

fclose(fid);
fprintf('relatorio escrito: %s (%d bytes)\n', arqSaida, getfield(dir(arqSaida),'bytes'));
end
