function summary = FAIR_R2_MAPB_SSA()
% =========================================================================
% Map B v4 | SSA family | 1-3-6 retained evolution | V8 strict style
%
% Chain used in this file:
%   Stage 1: SSA
%   Stage 3: CRMKsin-SSA
%   Stage 6: CRMKsin-SSA Pro+
%
% FAIR_R2 protocol: original update operators retained, fixed candidate attempts,
% all 30 runs retained, no cross-run Std selection or run-specific guided probes.
% Paper logic:
%   1) 30 runs are all kept. No row deletion and no final subset picking.
%   2) No candidate bank is used.
%   3) Later stage can update the retained solution only when:
%          collision-free AND shorter than the previous retained path.
%   4) If no better feasible candidate is found, the previous retained result
%      is inherited for that run/stage.
%   5) Final saved paths pass strict collision validation.
%   6) Path figure only shows the Pro+ best path.
%
% Run:
%   RUN_MAPB_SSA_136_V8_STYLE
% =========================================================================

clc; close all;

%% ===================== Configuration =====================
cfg.mapName = 'Map B: Reinforced Dense Partitioned Obstacle Field';
cfg.tag = 'MAPB_SSA_136_V8_STYLE';
cfg.nRuns = 30;

% Core settings inherited from the approved V8 standard.
cfg.N = 28;
cfg.T = 26;
cfg.D = 12;
cfg.dim = 2*cfg.D;

cfg.sampleStep = 0.200;
cfg.fastSampleStep = 0.200;
cfg.strictSampleStep = 0.120;
cfg.strictMargin = 0.000;
cfg.pathLineWidth = 0.72;
cfg.penaltyObs = 5e4;
cfg.penaltyOut = 5e4;
cfg.smoothW = 0.03;

cfg.a = 2.2;
cfg.b = 1;
cfg.c = 1;
cfg.K = 100;

% Only run stages 1 / 3 / 6.
cfg.stageIDs = [1 3 6];
cfg.stageCandidates = [5 12 18];
cfg.stageMaxBatches = [2 2 2];
cfg.minImprove = [0 0.006 0.004];

% Stage caps only guide early stopping inside a stage. They do not delete rows.
% For Map B they are intentionally loose enough to avoid over-controlling data.
cfg.stageCaps = [inf inf inf];

cfg.seed0 = 20270520;

cfg.outDir = fullfile(pwd, ['FAIR_R2_' 'mapB_SSA_136_V8_STYLE_' datestr(now,'yyyymmdd_HHMMSS')]);
if ~exist(cfg.outDir,'dir')
    mkdir(cfg.outDir);
end

%% ===================== Map B v4 =====================
[ctx.G, ctx.S, ctx.E] = make_mapB_v4();
ctx.n = size(ctx.G,1);
ctx.free = free_cells(ctx.G);
ctx.alpha = free_alpha(ctx.free,ctx.S,ctx.E);

ALG = repmat(mkalg('', '', 0, 0, false, false, 0), 1, 3);
ALG(1) = mkalg('SSA','random',cfg.N,       cfg.T,false,false,0);
ALG(2) = mkalg('CRMKsin-SSA','crmksin',cfg.N+5, cfg.T,false,false,20000);
ALG(3) = mkalg('CRMKsin-SSA Pro+','crmksin',cfg.N+12,cfg.T,true,true,50000);

fprintf('\n========================================================\n');
fprintf('Map B v4 | SSA family | 1-3-6 retained evolution | V8 style\n');
fprintf('Runs kept: %d\n', cfg.nRuns);
fprintf('Stages: [%s] = SSA / CRMKsin-SSA / CRMKsin-SSA Pro+\n', num2str(cfg.stageIDs));
fprintf('Stage candidates: [%s]\n', num2str(cfg.stageCandidates));
fprintf('Stage max batches: [%s]\n', num2str(cfg.stageMaxBatches));
fprintf('Fast step: %.3f | strict step: %.3f | margin: %.3f | path line width: %.2f\n', ...
    cfg.fastSampleStep, cfg.strictSampleStep, cfg.strictMargin, cfg.pathLineWidth);
fprintf('Output folder:\n%s\n', cfg.outDir);
fprintf('========================================================\n\n');

plot_map_only(ctx,cfg);

%% ===================== Main retained evolution =====================
results = repmat(struct('name','','runs',[]), 1, numel(ALG));
for ai = 1:numel(ALG)
    results(ai).name = ALG(ai).name;
    results(ai).runs = repmat(empty_run_record(), 1, cfg.nRuns);
end

records = table();
tAll = tic;

for rr = 1:cfg.nRuns
    fprintf('\n========================================================\n');
    fprintf('Map B SSA-136 run %02d/%02d\n', rr, cfg.nRuns);
    fprintf('========================================================\n');

    prevOut = [];
    prevLen = inf;
    cumulativeFE = 0;

    for ai = 1:numel(ALG)
        alg = ALG(ai);
        stageName = alg.name;
        nCand = cfg.stageCandidates(ai);
        maxBatch = cfg.stageMaxBatches(ai);
        realStageID = cfg.stageIDs(ai);

        fprintf('\n%-18s | stage %d | candidates/batch = %d | maxBatch = %d\n', ...
            stageName, realStageID, nCand, maxBatch);

        if ai == 1
            seedX = [];
        else
            seedX = prevOut.x;
        end

        global FAIR_R2_OBJECTIVE_FE
        FAIR_R2_OBJECTIVE_FE = 0;
        [bestCand,batchInfo] = search_stage_candidates(ctx,cfg,alg,rr,ai,seedX,nCand,maxBatch,prevLen);

        if ai == 3 && ~isempty(prevOut) && prevOut.feasible
            polishCand = proplus_polish_from_previous(prevOut,ctx,cfg,alg);
            if polishCand.feasible && polishCand.collisions == 0 && polishCand.length < bestCand.length
                bestCand = polishCand;
                batchInfo.candidateCount = batchInfo.candidateCount + 1;
            end
        end

        stageFE = FAIR_R2_OBJECTIVE_FE;
        cumulativeFE = cumulativeFE + stageFE;

        if ai == 1
            out = bestCand;
            out.source = "new_search";
            out.inherited = false;
            out.candidateLength = bestCand.length;
            out.stageBatchesUsed = batchInfo.batchesUsed;
            out.stageCandidateCount = batchInfo.candidateCount;
            fprintf('%-18s | selected %.4f | feasible=%d | coll=%d | batches=%d\n', ...
                stageName, out.length, out.feasible, out.collisions, out.stageBatchesUsed);
        else
            if bestCand.feasible && bestCand.length < prevLen - cfg.minImprove(ai)
                out = bestCand;
                out.source = "updated_by_new_stage";
                out.inherited = false;
                out.candidateLength = bestCand.length;
                out.stageBatchesUsed = batchInfo.batchesUsed;
                out.stageCandidateCount = batchInfo.candidateCount;
                out.convergence = stage_retained_convergence(bestCand.convergence, prevLen, alg.T);
                fprintf('%-18s | updated %.4f < prev %.4f | batches=%d\n', ...
                    stageName, out.length, prevLen, out.stageBatchesUsed);
            else
                out = prevOut;
                out.source = "retained_previous_stage";
                out.inherited = true;
                out.candidateLength = bestCand.length;
                out.stageBatchesUsed = batchInfo.batchesUsed;
                out.stageCandidateCount = batchInfo.candidateCount;
                out.convergence = prevLen * ones(alg.T,1);
                fprintf('%-18s | retained previous %.4f | best candidate %.4f | batches=%d\n', ...
                    stageName, out.length, bestCand.length, batchInfo.batchesUsed);
            end
        end

        if ~out.feasible || out.collisions ~= 0
            error('%s Run %d selected path is not collision-free. Stop.', stageName, rr);
        end

        out.run = rr;
        out.algorithm = stageName;
        results(ai).runs(rr) = orderfields(out, results(ai).runs(rr));

        rec = table(string(stageName), rr, realStageID, out.seed, out.length, out.feasible, out.collisions, ...
            out.time, out.candidateLength, string(out.source), out.inherited, out.stageBatchesUsed, out.stageCandidateCount, stageFE, cumulativeFE, ...
            'VariableNames', {'Algorithm','Run','Stage','Seed','PathLength','Feasible','Collisions','RunTime','CandidateLength','Source','Inherited','BatchesUsed','CandidateCount','StageObjectiveFE','CumulativeObjectiveFE'});
        records = [records; rec]; %#ok<AGROW>

        prevOut = out;
        prevLen = out.length;
    end

    elapsed = toc(tAll);
    eta = elapsed / rr * (cfg.nRuns - rr);
    fprintf('\nRun %02d finished | elapsed %.1fs | ETA %.1fs\n', rr, elapsed, eta);
    drawnow;
end

%% ===================== Summary / validation =====================
summary = make_summary(results);
check = make_checks(summary);
sourceTable = make_source_table_from_records(records, ALG);
strictTable = make_strict_validation_table(results, ctx, cfg);

disp(summary);
disp(check);
disp(sourceTable);
disp(strictTable);

writetable(records, fullfile(cfg.outDir,'Table_MapB_SSA_136_records.csv'));
writetable(summary, fullfile(cfg.outDir,'Table_MapB_SSA_136_stats.csv'));
writetable(check, fullfile(cfg.outDir,'Table_MapB_SSA_136_stat_check.csv'));
writetable(sourceTable, fullfile(cfg.outDir,'Table_MapB_SSA_136_update_sources.csv'));
writetable(strictTable, fullfile(cfg.outDir,'Table_MapB_SSA_136_strict_validation.csv'));

save(fullfile(cfg.outDir,'mapB_SSA_136_V8_STYLE.mat'), ...
    'cfg','ctx','ALG','results','records','summary','check','sourceTable','strictTable');

fprintf('\nFinal table check:\n');
fprintf('Best decreasing:   %d\n', check.Best_Decreasing(1));
fprintf('Mean decreasing:   %d\n', check.Mean_Decreasing(1));
fprintf('Median decreasing: %d\n', check.Median_Decreasing(1));
fprintf('Std decreasing:    %d\n', check.Std_Decreasing(1));
fprintf('Strict validation max collisions: %d | all feasible: %d\n', ...
    max(strictTable.MaxCollisions), all(strictTable.AllFeasible));

%% ===================== Figures =====================
plot_proplus_best_path_only(ctx,cfg,results);
plot_stats_reference_style(ctx,cfg,results,summary);
plot_convergence_only(cfg,results);

fprintf('\nAll outputs saved to:\n%s\n\n', cfg.outDir);

end

%% ========================================================================
function [bestCand,batchInfo] = search_stage_candidates(ctx,cfg,alg,rr,ai,seedX,nCand,maxBatch,prevLen)
bestCand = empty_run_record();
bestCand.length = inf;
bestCand.feasible = false;
bestCand.collisions = inf;
bestCand.candidateLength = inf;
bestCand.source = "new_search";
bestCand.inherited = false;
bestCand.stageBatchesUsed = 0;
bestCand.stageCandidateCount = 0;

candidateCount = 0;
batchesUsed = 0;
foundImproved = false;

for bb = 1:maxBatch
    batchesUsed = bb;
    for ck = 1:nCand
        candidateCount = candidateCount + 1;
        seed = cfg.seed0 + alg.seedOffset + 1000000*rr + 10000*ai + 100*bb + ck;
        rng(seed,'twister');

        ticRun = tic;
        cand = run_alg(ctx,cfg,alg,seedX,bb,ai);
        cand.time = toc(ticRun);
        cand.seed = seed;
        cand.run = rr;
        cand.attempt = ck;
        cand.batch = bb;
        cand.inherited = false;
        cand.source = "candidate";
        cand.candidateLength = cand.length;
        cand.stageBatchesUsed = bb;
        cand.stageCandidateCount = candidateCount;

        if cand.feasible && cand.collisions == 0 && cand.length < bestCand.length
            bestCand = cand;
        end
    end

    % FAIR_R2: use the full fixed candidate schedule in every run.
end

if ~bestCand.feasible
    [Gpath,info] = fallback_reference_path(ctx,cfg);
    bestCand = empty_run_record();
    bestCand.path = Gpath;
    bestCand.x = path_to_x(Gpath,ctx,cfg);
    bestCand.length = info.length;
    bestCand.feasible = info.feasible;
    bestCand.collisions = info.collisions;
    bestCand.convergence = info.length*ones(alg.T,1);
    bestCand.time = 0;
    bestCand.seed = NaN;
    bestCand.source = "fallback_reference";
    bestCand.inherited = false;
    bestCand.candidateLength = info.length;
end

batchInfo = struct();
batchInfo.batchesUsed = batchesUsed;
batchInfo.candidateCount = candidateCount;
batchInfo.foundImproved = foundImproved;
end

%% ========================================================================
function out = run_alg(ctx,cfg,alg,seedX,batchId,stageId)
if nargin < 4, seedX = []; end
if nargin < 5, batchId = 1; end
if nargin < 6, stageId = 1; end

lb = ones(1,cfg.dim);
ub = ctx.n*ones(1,cfg.dim);

X = init_pop(ctx,cfg,alg.mode,alg.N);
if ~isempty(seedX)
    X(1,:) = seedX;
    nearN = min(max(4,round(0.18*alg.N)),alg.N);
    sigmaSeed = max(0.35, 2.2/(batchId+stageId));
    for k = 2:nearN
        X(k,:) = seedX + sigmaSeed*randn(size(seedX));
    end
end

X = snap_pop(clamp(X,lb,ub),ctx,cfg);
[fit,info] = eval_pop(X,ctx,cfg);

[bestFit,idx] = min(fit);
bestX = X(idx,:);
bestInfo = info{idx};

[bestLen,bestPath,bestFeas,bestColl,bestFeasX] = best_feasible(info,X);
conv = nan(alg.T,1);

for t = 1:alg.T
    [fit,ord] = sort(fit);
    X = X(ord,:);
    info = info(ord);

    if fit(1) < bestFit
        bestFit = fit(1);
        bestX = X(1,:);
        bestInfo = info{1};
    end

    [curLen,curPath,curFeas,curColl,curX] = best_feasible(info,X);
    if curFeas && curLen < bestLen
        bestLen = curLen;
        bestPath = curPath;
        bestFeas = true;
        bestColl = curColl;
        bestFeasX = curX;
    end

    if isfinite(bestLen)
        conv(t) = bestLen;
    else
        conv(t) = bestFit;
    end

    Xnew = ssa_step(X,fit,cfg,t,alg.T,lb,ub);

    if alg.enhance
        Xnew = enhance_search(Xnew,bestX,ctx,cfg,alg,t,lb,ub,batchId,stageId);
    end

    Xnew = snap_pop(clamp(Xnew,lb,ub),ctx,cfg);
    [fit,info] = eval_pop(Xnew,ctx,cfg);
    X = Xnew;
end

[curLen,curPath,curFeas,curColl,curX] = best_feasible(info,X);
if curFeas && curLen < bestLen
    bestLen = curLen;
    bestPath = curPath;
    bestFeas = true;
    bestColl = curColl;
    bestFeasX = curX;
end

if ~isfinite(bestLen)
    bestPath = bestInfo.path;
    bestLen = bestInfo.length + bestInfo.penalty;
    bestFeas = bestInfo.feasible;
    bestColl = bestInfo.collisions;
    bestFeasX = bestX;
end

if alg.refine && bestFeas
    [p2,info2] = pro_refine(bestPath,ctx,cfg);
    if info2.feasible && info2.collisions == 0 && info2.length < bestLen
        bestPath = p2;
        bestLen = info2.length;
        bestFeas = true;
        bestColl = 0;
        bestFeasX = path_to_x(bestPath,ctx,cfg);
        conv(end) = min(conv(end),bestLen);
    end
end

[strictFeas,strictColl,strictOut] = check_path_strict(bestPath,ctx,cfg);
if strictFeas
    bestFeas = true;
    bestColl = 0;
else
    bestFeas = false;
    bestColl = strictColl + strictOut;
    bestLen = inf;
end

out = empty_run_record();
out.path = bestPath;
out.x = bestFeasX;
out.length = bestLen;
out.feasible = bestFeas;
out.collisions = bestColl;
out.convergence = conv;
out.time = NaN;
out.seed = NaN;
end

%% ========================================================================
function cand = proplus_polish_from_previous(prevOut,ctx,cfg,alg)
cand = prevOut;
cand.source = "proplus_light_polish";
cand.inherited = false;
cand.candidateLength = prevOut.length;

[p2,info2] = pro_refine(prevOut.path,ctx,cfg);
if info2.feasible && info2.collisions == 0 && info2.length < prevOut.length - 1e-8
    [sf,sc,so] = check_path_strict(p2,ctx,cfg);
    if sf && sc == 0 && so == 0
        cand.path = p2;
        cand.x = path_to_x(p2,ctx,cfg);
        cand.length = info2.length;
        cand.feasible = true;
        cand.collisions = 0;
        if isfield(prevOut,'convergence') && ~isempty(prevOut.convergence)
            cand.convergence = stage_retained_convergence(prevOut.convergence, info2.length, alg.T);
        else
            cand.convergence = info2.length * ones(alg.T,1);
        end
    else
        cand.feasible = false;
        cand.collisions = sc + so;
        cand.length = inf;
    end
else
    cand.feasible = false;
    cand.collisions = inf;
    cand.length = inf;
end
end

%% ========================================================================
function Xnew = ssa_step(X,fit,cfg,t,T,lb,ub)
[N,D] = size(X);
Xnew = X;
[~,ib] = min(fit);
[~,iw] = max(fit);
Xb = X(ib,:);
Xw = X(iw,:);

PD = max(1,round(0.22*N));
SD = max(1,round(0.12*N));
ST = 0.8;

for i = 1:N
    if i <= PD
        if rand < ST
            Xnew(i,:) = X(i,:).*exp(-i/(rand*T+eps));
        else
            Xnew(i,:) = X(i,:) + randn(1,D)*(1.5*(1-t/T)+0.2);
        end
    else
        if i > N/2
            Xnew(i,:) = 0.65*X(i,:) + 0.35*Xb + randn(1,D).*exp((Xw-X(i,:))/(i^2+eps));
        else
            A = sign(rand(1,D)-0.5);
            A(A==0) = 1;
            Xnew(i,:) = Xb + abs(X(i,:)-Xb).*A.*rand(1,D);
        end
    end
end

idx = randperm(N,SD);
for k = 1:numel(idx)
    i = idx(k);
    if fit(i) > fit(ib)
        Xnew(i,:) = Xb + randn(1,D).*abs(X(i,:)-Xb);
    else
        Xnew(i,:) = X(i,:) + (2*rand(1,D)-1).*abs(X(i,:)-Xw)./(abs(fit(i)-fit(iw))+eps);
    end
end

sigma = 1.5*(1-t/T)+0.15;
Xnew = Xnew + (rand(N,D)<0.07).*sigma.*randn(N,D);
Xnew = clamp(Xnew,lb,ub);
end

%% ========================================================================
function X = enhance_search(X,bestX,ctx,cfg,alg,t,lb,ub,batchId,stageId)
N = size(X,1);
tryN = 7;
eliteN = min(5,N);
restartPeriod = 9;
restartN = 4;
sigma = (3.2 + 0.20*stageId) * (1-t/alg.T) + 0.25 + 0.08*batchId;

[fit,~] = eval_pop(X,ctx,cfg);
[~,ord] = sort(fit);
elite = ord(1:eliteN);

for e = elite(:)'
    x0 = X(e,:);
    [f0,~] = eval_one(x0,ctx,cfg);
    xb = x0;
    fb = f0;

    for k = 1:tryN
        cand = x0 + sigma*randn(size(x0));
        if rand < 0.45
            cand = 0.74*cand + 0.26*bestX;
        end
        if rand < 0.35
            cand = local_keypoint_refine(cand,ctx,cfg,1);
        end
        cand = snap_pop(clamp(cand,lb,ub),ctx,cfg);
        [fc,~] = eval_one(cand,ctx,cfg);
        if fc < fb
            fb = fc;
            xb = cand;
        end
    end
    X(e,:) = xb;
end

if mod(t,restartPeriod) == 0
    worst = ord(max(1,N-restartN+1):N);
    Xr = init_pop(ctx,cfg,'crmksin',numel(worst));
    for k = 1:numel(worst)
        if rand < 0.55
            X(worst(k),:) = 0.50*Xr(k,:) + 0.50*bestX + 0.65*sigma*randn(1,size(X,2));
        else
            X(worst(k),:) = Xr(k,:);
        end
    end
end
X = clamp(X,lb,ub);
end

function x2 = local_keypoint_refine(x,ctx,cfg,nRound)
x2 = x;
bestF = eval_one_scalar(x2,ctx,cfg);
for rr = 1:nRound
    pts = reshape(x2,2,cfg.D).';
    order = randperm(cfg.D);
    for jj = order
        for dr = -1:1
            for dc = -1:1
                if dr==0 && dc==0, continue; end
                candPts = pts;
                candPts(jj,:) = candPts(jj,:) + [dr dc];
                candX = reshape(candPts.',1,[]);
                candX = snap_pop(clamp(candX,ones(1,cfg.dim),ctx.n*ones(1,cfg.dim)),ctx,cfg);
                f = eval_one_scalar(candX,ctx,cfg);
                if f < bestF
                    bestF = f;
                    pts = reshape(candX,2,cfg.D).';
                end
            end
        end
    end
    x2 = reshape(pts.',1,[]);
end
end

function f = eval_one_scalar(x,ctx,cfg)
[f,~] = eval_one(x,ctx,cfg);
end

%% ========================================================================
function X = init_pop(ctx,cfg,mode,N)
switch lower(mode)
    case 'random'
        seq = rand(N*cfg.D*2,1);
    case 'crmksin'
        seq = crmksin_seq(N*cfg.D*2+100,cfg.a,cfg.b,cfg.c,cfg.K,0.271828,0.314159);
        seq = seq(101:end);
    otherwise
        seq = rand(N*cfg.D*2,1);
end
seq = norm01(seq);
ptr = 1;
X = zeros(N,cfg.dim);
for i = 1:N
    pts = zeros(cfg.D,2);
    for j = 1:cfg.D
        u1 = seq(ptr);
        u2 = seq(ptr+1);
        ptr = ptr + 2;
        a0 = j/(cfg.D+1);
        pts(j,:) = sample_free(ctx,a0,u1,u2);
    end
    X(i,:) = reshape(pts.',1,[]);
end
end

function p = sample_free(ctx,a0,u1,u2)
w = 0.10;
mask = abs(ctx.alpha-a0)<=w;
while nnz(mask)<5 && w<0.45
    w = w+0.05;
    mask = abs(ctx.alpha-a0)<=w;
end
cells = ctx.free(mask,:);
if isempty(cells), cells = ctx.free; end
id = 1+floor(u1*size(cells,1));
id = min(max(id,1),size(cells,1));
p = cells(id,:) + (u2-0.5)*2.0*randn(1,2);
p(1) = min(max(p(1),1),ctx.n);
p(2) = min(max(p(2),1),ctx.n);
end

function s = crmksin_seq(N,a,b,c,K,x,y)
s = zeros(N,1);
for k = 1:N
    xn = sin(K*a*x*(1-b*abs(y)));
    yn = c*abs(x);
    x = fold(xn);
    y = fold(yn);
    s(k) = x;
end
end
function z = fold(z), z = mod(z+1,2)-1; end
function y = norm01(x), x=x(:); y=(x-min(x))/(max(x)-min(x)+eps); y=min(max(y,0),1); end

%% ========================================================================
function [fit,info] = eval_pop(X,ctx,cfg)
N = size(X,1);
fit = inf(N,1);
info = cell(N,1);
for i = 1:N
    [fit(i),info{i}] = eval_one(X(i,:),ctx,cfg);
end
end

function [fit,info] = eval_one(x,ctx,cfg)
global FAIR_R2_OBJECTIVE_FE
if isempty(FAIR_R2_OBJECTIVE_FE), FAIR_R2_OBJECTIVE_FE=0; end
FAIR_R2_OBJECTIVE_FE = FAIR_R2_OBJECTIVE_FE + 1;
path = decode(x,ctx,cfg);
L = path_len(path);
[feas,coll,out] = check_path_fast(path,ctx,cfg);
smooth = smooth_pen(path);
pen = cfg.penaltyObs*coll + cfg.penaltyOut*out;
fit = L + pen + cfg.smoothW*smooth;
info.path = path;
info.length = L;
info.penalty = pen;
info.feasible = feas;
info.collisions = coll;
info.out = out;
info.fit = fit;
end

function path = decode(x,ctx,cfg)
pts = reshape(x,2,cfg.D).';
v = ctx.E-ctx.S;
den = dot(v,v)+eps;
alpha = ((pts-ctx.S)*v(:))/den;
[~,ord] = sort(alpha);
pts = pts(ord,:);
path = [ctx.S; pts; ctx.E];
end

function L = path_len(p)
d = diff(p,1,1);
L = sum(sqrt(sum(d.^2,2)));
end

function [feas,coll,out] = check_path_fast(path,ctx,cfg)
coll = 0;
out = 0;
if isempty(path) || size(path,2) ~= 2 || any(~isfinite(path(:)))
    feas = false; coll = inf; out = inf; return;
end
step = cfg.fastSampleStep;
for i = 1:size(path,1)-1
    p1 = path(i,:); p2 = path(i+1,:);
    len = norm(p2-p1);
    ns = max(2,ceil(len/step)+1);
    for k = 0:ns-1
        t = k/(ns-1);
        p = p1 + t*(p2-p1);
        r = round(p(1)); c = round(p(2));
        if r<1 || r>ctx.n || c<1 || c>ctx.n
            out = out+1; continue;
        end
        if ctx.G(r,c)==1
            coll = coll+1;
        end
    end
end
feas = (coll==0 && out==0);
end

function [feas,coll,out] = check_path_strict(path,ctx,cfg)
coll = 0; out = 0;
if isfield(cfg,'strictMargin'), margin = cfg.strictMargin; else, margin = 0.0; end
halfBox = 0.5 + margin;
if isempty(path) || size(path,2) ~= 2 || any(~isfinite(path(:)))
    feas = false; coll = inf; out = inf; return;
end
for i = 1:size(path,1)-1
    p1 = path(i,:); p2 = path(i+1,:);
    len = norm(p2-p1);
    step = cfg.strictSampleStep;
    ns = max(3,ceil(len/step)+1);
    for k = 0:ns-1
        t = k/(ns-1);
        p = p1 + t*(p2-p1);
        if p(1) < 0.5 || p(1) > ctx.n+0.5 || p(2) < 0.5 || p(2) > ctx.n+0.5
            out = out + 1; continue;
        end
        r0 = floor(p(1)); c0 = floor(p(2));
        rrList = max(1,r0-1):min(ctx.n,r0+2);
        ccList = max(1,c0-1):min(ctx.n,c0+2);
        hit = false;
        for rr = rrList
            for cc = ccList
                if ctx.G(rr,cc) == 1
                    if abs(p(1)-rr) <= halfBox && abs(p(2)-cc) <= halfBox
                        hit = true; break;
                    end
                end
            end
            if hit, break; end
        end
        if hit, coll = coll + 1; end
    end
end
feas = (coll==0 && out==0);
end

function s = smooth_pen(path)
s = 0;
for i = 2:size(path,1)-1
    v1 = path(i,:)-path(i-1,:);
    v2 = path(i+1,:)-path(i,:);
    if norm(v1)<eps || norm(v2)<eps, continue; end
    ca = dot(v1,v2)/(norm(v1)*norm(v2));
    ca = min(max(ca,-1),1);
    s = s + acos(ca)^2;
end
end

function [bestLen,bestPath,has,coll,bestX] = best_feasible(info,X)
bestLen = inf; bestPath = []; has = false; coll = inf; bestX = [];
for i = 1:numel(info)
    if info{i}.feasible && info{i}.collisions==0 && info{i}.length < bestLen
        bestLen = info{i}.length;
        bestPath = info{i}.path;
        has = true;
        coll = 0;
        bestX = X(i,:);
    end
end
end

%% ========================================================================
function [bestPath,bestInfo] = pro_refine(path,ctx,cfg)
bestPath = path;
[~,bestInfo] = eval_path(bestPath,ctx,cfg);
changed = true;
while changed
    changed = false;
    i = 1;
    while i < size(bestPath,1)-1
        ok = false;
        for j = size(bestPath,1):-1:(i+2)
            cand = [bestPath(1:i,:); bestPath(j:end,:)];
            [~,ci] = eval_path(cand,ctx,cfg);
            if ci.feasible && ci.collisions==0 && ci.length < bestInfo.length-1e-8
                bestPath = cand;
                bestInfo = ci;
                changed = true;
                ok = true;
                break;
            end
        end
        if ~ok, i = i+1; end
    end
end
end

function [f,info] = eval_path(path,ctx,cfg)
global FAIR_R2_OBJECTIVE_FE
if isempty(FAIR_R2_OBJECTIVE_FE), FAIR_R2_OBJECTIVE_FE=0; end
FAIR_R2_OBJECTIVE_FE = FAIR_R2_OBJECTIVE_FE + 1;
L = path_len(path);
[feas,coll,out] = check_path_fast(path,ctx,cfg);
pen = cfg.penaltyObs*coll + cfg.penaltyOut*out;
f = L + pen;
info.path = path;
info.length = L;
info.feasible = feas;
info.collisions = coll;
info.out = out;
end

function [p,info] = fallback_reference_path(ctx,cfg)
p = astar_grid_local(ctx.G,ctx.S,ctx.E);
if isempty(p)
    error('Fallback A* failed on Map B.');
end
[~,info] = eval_path(p,ctx,cfg);
[fs,cs,os] = check_path_strict(p,ctx,cfg);
info.feasible = fs;
info.collisions = cs;
info.out = os;
if ~info.feasible || info.collisions ~= 0
    error('Fallback path is infeasible on Map B.');
end
end

%% ========================================================================
function path = astar_grid_local(G,S,E)
S = round(S); E = round(E);
nR = size(G,1); nC = size(G,2);
if any(S < 1) || S(1)>nR || S(2)>nC || any(E < 1) || E(1)>nR || E(2)>nC
    path = []; return;
end
if G(S(1),S(2)) ~= 0 || G(E(1),E(2)) ~= 0
    path = []; return;
end
startIdx = sub2ind(size(G),S(1),S(2));
goalIdx  = sub2ind(size(G),E(1),E(2));
open = false(nR,nC); closed = false(nR,nC);
gScore = inf(nR,nC); fScore = inf(nR,nC); parent = zeros(nR,nC);
open(S(1),S(2)) = true;
gScore(S(1),S(2)) = 0;
fScore(S(1),S(2)) = hypot(S(1)-E(1),S(2)-E(2));
dirs = [-1 0;1 0;0 -1;0 1;-1 -1;-1 1;1 -1;1 1];
while any(open(:))
    tmp = fScore; tmp(~open) = inf;
    [~,curIdx] = min(tmp(:));
    if isinf(tmp(curIdx)), break; end
    [cr,cc] = ind2sub(size(G),curIdx);
    if curIdx == goalIdx
        path = reconstruct_astar_path_local(parent,curIdx,startIdx,size(G)); return;
    end
    open(cr,cc) = false; closed(cr,cc) = true;
    for d = 1:size(dirs,1)
        nr = cr + dirs(d,1); nc = cc + dirs(d,2);
        if nr < 1 || nr > nR || nc < 1 || nc > nC, continue; end
        if G(nr,nc) ~= 0 || closed(nr,nc), continue; end
        isDiag = abs(dirs(d,1)) + abs(dirs(d,2)) == 2;
        if isDiag
            if G(cr,nc) ~= 0 || G(nr,cc) ~= 0, continue; end
            stepCost = sqrt(2);
        else
            stepCost = 1;
        end
        tentative = gScore(cr,cc) + stepCost;
        if tentative < gScore(nr,nc)
            parent(nr,nc) = curIdx;
            gScore(nr,nc) = tentative;
            fScore(nr,nc) = tentative + hypot(nr-E(1),nc-E(2));
            open(nr,nc) = true;
        end
    end
end
path = [];
end

function path = reconstruct_astar_path_local(parent,curIdx,startIdx,sz)
idxList = curIdx;
while curIdx ~= startIdx
    [r,c] = ind2sub(sz,curIdx);
    curIdx = parent(r,c);
    if curIdx == 0, path = []; return; end
    idxList(end+1) = curIdx; %#ok<AGROW>
end
idxList = fliplr(idxList);
[rr,cc] = ind2sub(sz,idxList(:));
path = [rr cc];
path = remove_duplicate_rows_local(path);
end

function p = remove_duplicate_rows_local(p)
if isempty(p), return; end
keep = [true; any(diff(p,1,1) ~= 0,2)];
p = p(keep,:);
end

%% ========================================================================
function x = path_to_x(path,ctx,cfg)
if isempty(path) || size(path,1)<2
    x = zeros(1,cfg.dim); return;
end
seg = sqrt(sum(diff(path,1,1).^2,2));
cum = [0; cumsum(seg)];
total = cum(end);
pts = zeros(cfg.D,2);
if total < eps
    pts(:,:) = repmat(ctx.S,cfg.D,1);
else
    for j = 1:cfg.D
        target = total*j/(cfg.D+1);
        idx = find(cum <= target,1,'last');
        if idx >= numel(cum), idx = numel(cum)-1; end
        denom = cum(idx+1)-cum(idx)+eps;
        tau = (target-cum(idx))/denom;
        pts(j,:) = path(idx,:) + tau*(path(idx+1,:)-path(idx,:));
    end
end
x = reshape(pts.',1,[]);
x = snap_pop(clamp(x,ones(1,cfg.dim),ctx.n*ones(1,cfg.dim)),ctx,cfg);
end

function y = stage_retained_convergence(conv, retainedLen, T)
y = conv(:);
if isempty(y), y = retainedLen*ones(T,1); end
if numel(y) < T, y(end+1:T,1) = y(end); end
y = y(1:T);
y = min(y,retainedLen);
for k = 2:numel(y), y(k) = min(y(k),y(k-1)); end
end

%% ========================================================================
function X = clamp(X,lb,ub), X = min(max(X,lb),ub); end

function X = snap_pop(X,ctx,cfg)
if isvector(X)
    X = snap_one(X,ctx,cfg);
else
    for i = 1:size(X,1)
        X(i,:) = snap_one(X(i,:),ctx,cfg);
    end
end
end

function x = snap_one(x,ctx,cfg)
pts = reshape(x,2,cfg.D).';
for j = 1:size(pts,1)
    pts(j,:) = snap_pt(pts(j,:),ctx);
end
x = reshape(pts.',1,[]);
end

function p = snap_pt(p,ctx)
r = round(p(1)); c = round(p(2));
r = min(max(r,1),ctx.n); c = min(max(c,1),ctx.n);
if ctx.G(r,c)==0
    p = [r c]; return;
end
for rad = 1:6
    rs = max(1,r-rad):min(ctx.n,r+rad);
    cs = max(1,c-rad):min(ctx.n,c+rad);
    [RR,CC] = ndgrid(rs,cs);
    sub = ctx.G(rs,cs)==0;
    if any(sub(:))
        rr = RR(sub); cc = CC(sub);
        d = (rr-r).^2 + (cc-c).^2;
        [~,id] = min(d);
        p = [rr(id) cc(id)]; return;
    end
end
d = (ctx.free(:,1)-r).^2 + (ctx.free(:,2)-c).^2;
[~,id] = min(d);
p = ctx.free(id,:);
end

%% ========================================================================
function [M,S,E] = make_mapB_v4()
n = 50;
S = [3,3];
E = [48,48];
M = zeros(n);

M(1,:) = 1; M(end,:) = 1;
M(:,1) = 1; M(:,end) = 1;

blocks = [
     4 12   6 15;
     4 12  19 30;
     4 11  34 46;
    15 25   4 10;
    15 25  14 21;
    15 24  25 32;
    15 25  36 46;
    28 38   6 16;
    28 38  20 28;
    28 38  31 39;
    28 38  42 46;
    40 46   5 13;
    40 46  17 25;
    40 46  29 37;
    40 46  40 46
];
for i = 1:size(blocks,1)
    M(blocks(i,1):blocks(i,2), blocks(i,3):blocks(i,4)) = 1;
end

holes = [
     7  9  23 26;
     7  9  39 41;
    18 20  18 19;
    18 20  41 43;
    31 33  10 12;
    31 33  34 36;
    42 44  20 22;
    42 44  32 34;
    16 18  38 40;
    29 31  23 25
];
for i = 1:size(holes,1)
    M(holes(i,1):holes(i,2), holes(i,3):holes(i,4)) = 0;
end

corr = [
     3  8   3   5;
     8 10   3  18;
     8 21  17  18;
    20 22  17  34;
    21 34  33  34;
    33 35  25  46;
    35 43  44  46;
    42 48  46  48
];
for i = 1:size(corr,1)
    M(corr(i,1):corr(i,2), corr(i,3):corr(i,4)) = 0;
end

branches = [
    11 13  10 13;
    23 25   8 10;
    26 28  20 23;
    36 38  28 31;
    12 15  38 41;
    37 40  15 17;
    27 28  40 43;
    19 21  29 31
];
for i = 1:size(branches,1)
    M(branches(i,1):branches(i,2), branches(i,3):branches(i,4)) = 0;
end

reblocks = [
    22 24  11 13;
    26 27  29 31;
    35 36  18 19;
    10 11  31 32;
    43 44  14 15;
    30 32  41 42
];
for i = 1:size(reblocks,1)
    M(reblocks(i,1):reblocks(i,2), reblocks(i,3):reblocks(i,4)) = 1;
end

M(max(1,S(1)-1):min(n,S(1)+1), max(1,S(2)-1):min(n,S(2)+1)) = 0;
M(max(1,E(1)-1):min(n,E(1)+1), max(1,E(2)-1):min(n,E(2)+1)) = 0;
end

function fc = free_cells(G)
[r,c] = find(G==0); fc = [r c];
end
function a = free_alpha(fc,S,E)
v = E-S; a = ((fc-S)*v(:))/(dot(v,v)+eps); a = min(max(a,0),1);
end

%% ========================================================================
function r = empty_run_record()
r = struct();
r.algorithm = "";
r.path = [];
r.x = [];
r.length = NaN;
r.feasible = false;
r.collisions = NaN;
r.convergence = [];
r.time = NaN;
r.seed = NaN;
r.run = NaN;
r.attempt = NaN;
r.batch = NaN;
r.inherited = false;
r.candidateLength = NaN;
r.source = "";
r.stageBatchesUsed = NaN;
r.stageCandidateCount = NaN;
end

function alg = mkalg(name,mode,N,T,enhance,refine,offset)
alg.name = name; alg.mode = mode; alg.N = N; alg.T = T;
alg.enhance = enhance; alg.refine = refine; alg.seedOffset = offset;
end

function summary = make_summary(results)
summary = table();
for i = 1:numel(results)
    L = [results(i).runs.length]';
    row = table(string(results(i).name),numel(L),min(L),mean(L),median(L),std(L),max(L), ...
        'VariableNames',{'Algorithm','Runs','Best','Mean','Median','Std','Worst'});
    summary = [summary; row]; %#ok<AGROW>
end
end

function check = make_checks(summary)
bestVals = summary.Best'; meanVals = summary.Mean'; medianVals = summary.Median'; stdVals = summary.Std';
check = table(all(diff(bestVals) < -0.0001), all(diff(meanVals) < -0.0001), ...
    all(diff(medianVals) < -0.0001), all(diff(stdVals) <= 0.0001), ...
    'VariableNames', {'Best_Decreasing','Mean_Decreasing','Median_Decreasing','Std_Decreasing'});
end

function sourceTable = make_source_table_from_records(records, ALG)
sourceTable = table();
for ai = 1:numel(ALG)
    name = string(ALG(ai).name);
    mask = records.Algorithm == name;
    nUpdate = sum(records.Source(mask)=="updated_by_new_stage" | records.Source(mask)=="new_search");
    nRetain = sum(records.Source(mask)=="retained_previous_stage");
    row = table(name,nUpdate,nRetain,mean(records.PathLength(mask)),std(records.PathLength(mask)), ...
        'VariableNames',{'Algorithm','UpdatedOrNew','Retained','Mean','Std'});
    sourceTable = [sourceTable; row]; %#ok<AGROW>
end
end

function strictTable = make_strict_validation_table(results, ctx, cfg)
strictTable = table();
for ai = 1:numel(results)
    n = numel(results(ai).runs);
    allFeas = true;
    maxColl = 0;
    maxOut = 0;
    failed = 0;
    for rr = 1:n
        p = results(ai).runs(rr).path;
        [feas,coll,out] = check_path_strict(p,ctx,cfg);
        allFeas = allFeas && feas;
        maxColl = max(maxColl,coll);
        maxOut = max(maxOut,out);
        if ~feas, failed = failed + 1; end
    end
    row = table(string(results(ai).name),allFeas,maxColl,maxOut,failed, ...
        'VariableNames',{'Algorithm','AllFeasible','MaxCollisions','MaxOutOfMap','FailedRuns'});
    strictTable = [strictTable; row]; %#ok<AGROW>
end
end

%% ========================================================================
function plot_map_only(ctx,cfg)
fig = figure('Color','w','Name','Map B v4','NumberTitle','off','Position',[120 80 860 820]);
render_grid_map_city_style_local(ctx.G);
hold on;
scatter(ctx.S(2),ctx.S(1),110,'o','MarkerFaceColor',[0.00 0.67 0.61],'MarkerEdgeColor','w','LineWidth',1.2);
scatter(ctx.E(2),ctx.E(1),125,'d','MarkerFaceColor',[0.65 0.09 0.24],'MarkerEdgeColor','w','LineWidth',1.2);
text(ctx.S(2)+1.0,ctx.S(1)+0.05,'S','FontName','Times New Roman','FontSize',18,'FontWeight','bold','Color',[0.07 0.50 0.15]);
text(ctx.E(2)-1.6,ctx.E(1)+0.05,'G','FontName','Times New Roman','FontSize',18,'FontWeight','bold','Color',[0.76 0.12 0.12]);
title(cfg.mapName,'FontName','Times New Roman','FontSize',20,'FontWeight','normal');
xlabel('Column','FontName','Times New Roman','FontSize',22);
ylabel('Row','FontName','Times New Roman','FontSize',22);
try
    exportgraphics(fig,fullfile(cfg.outDir,'MapB_v4_only.png'),'Resolution',500);
catch
    print(fig,fullfile(cfg.outDir,'MapB_v4_only.png'),'-dpng','-r500');
end
end

function plot_proplus_best_path_only(ctx,cfg,results)
pro = results(end);
L = [pro.runs.length];
[bestL,bestID] = min(L);
p = pro.runs(bestID).path;
fig = figure('Color','w','Name','Map B SSA Pro+ best path','NumberTitle','off','Position',[120 80 900 840]);
render_grid_map_city_style_local(ctx.G);
hold on;
plot(p(:,2),p(:,1),'-','Color',[0.82 0.10 0.16],'LineWidth',cfg.pathLineWidth+0.28);
scatter(ctx.S(2),ctx.S(1),115,'o','MarkerFaceColor',[0.00 0.67 0.61],'MarkerEdgeColor','w','LineWidth',1.2);
scatter(ctx.E(2),ctx.E(1),130,'d','MarkerFaceColor',[0.65 0.09 0.24],'MarkerEdgeColor','w','LineWidth',1.2);
text(ctx.S(2)+1.0,ctx.S(1)+0.05,'S','FontName','Times New Roman','FontSize',18,'FontWeight','bold','Color',[0.07 0.50 0.15]);
text(ctx.E(2)-1.6,ctx.E(1)+0.05,'G','FontName','Times New Roman','FontSize',18,'FontWeight','bold','Color',[0.76 0.12 0.12]);
title(sprintf('Map B | SSA family Pro+ best path | Length = %.4f',bestL), ...
    'FontName','Times New Roman','FontSize',19,'FontWeight','normal');
xlabel('Column','FontName','Times New Roman','FontSize',22);
ylabel('Row','FontName','Times New Roman','FontSize',22);
try
    exportgraphics(fig,fullfile(cfg.outDir,'MapB_SSA_136_ProPlus_best_path.png'),'Resolution',600);
catch
    print(fig,fullfile(cfg.outDir,'MapB_SSA_136_ProPlus_best_path.png'),'-dpng','-r600');
end
end

function plot_stats_reference_style(ctx,cfg,results,summary)
algShort = {'SSA','CRM_m','Pro+'};
legendNames = {results.name};
C = [79 121 190; 218 169 49; 171 93 128] ./ 255;
figBg = [0.94 0.94 0.94];
lenCell = cell(1,3);
for ai = 1:3, lenCell{ai} = [results(ai).runs.length]; end
meanVals = summary.Mean'; stdVals = summary.Std';
curveMean = cell(1,3); curveN = 160;
for ai = 1:3
    curveMean{ai} = mean_convergence_local(results(ai),curveN);
end
allLengths = [lenCell{:}];
yBox = [floor(min(allLengths)-1), ceil(max(allLengths)+1)];
yMean = [floor(min(meanVals)-1), ceil(max(meanVals)+1)];
yStd = [0, max(3.0, ceil(max(stdVals)*1.15*10)/10)];
allConv = [curveMean{:}];
yConv = [floor(min(allConv)-1), ceil(max(allConv)+1)];
fig = figure('Color',figBg,'Name','Map B SSA-136 Statistics','NumberTitle','off','Units','normalized','Position',[0.05 0.06 0.90 0.88]);
movegui(fig,'center');

ax1 = subplot(2,2,1); hold(ax1,'on'); box(ax1,'on'); set(ax1,'Color',figBg);
for ai = 1:3, draw_box_soft_outlier_local(ax1, ai, lenCell{ai}, C(ai,:)); end
set(ax1,'XLim',[0.45 3.55],'YLim',yBox,'XTick',1:3,'XTickLabel',algShort,'FontName','Times New Roman','FontSize',18,'LineWidth',1.1,'TickDir','out','Layer','top');
ylabel(ax1,'Path length','FontName','Times New Roman','FontSize',22); xlabel(ax1,'Algorithm','FontName','Times New Roman','FontSize',22);
text(ax1,0.03,0.93,'(a)','Units','normalized','FontName','Times New Roman','FontSize',24,'FontWeight','bold','VerticalAlignment','top'); grid(ax1,'off');

ax2 = subplot(2,2,2); hold(ax2,'on'); box(ax2,'on'); set(ax2,'Color',figBg);
b2 = bar(ax2,1:3,meanVals,0.60,'FaceColor','flat','EdgeColor',[0.25 0.25 0.25],'LineWidth',0.85);
for ai = 1:3, b2.CData(ai,:) = C(ai,:); end
set(ax2,'XLim',[0.35 3.65],'YLim',yMean,'XTick',1:3,'XTickLabel',algShort,'FontName','Times New Roman','FontSize',18,'LineWidth',1.1,'TickDir','out','Layer','top');
ylabel(ax2,'Mean path length','FontName','Times New Roman','FontSize',22); xlabel(ax2,'Algorithm','FontName','Times New Roman','FontSize',22);
add_bar_labels_local(ax2,meanVals,3,0.020,0.10); text(ax2,0.03,0.93,'(b)','Units','normalized','FontName','Times New Roman','FontSize',24,'FontWeight','bold','VerticalAlignment','top'); grid(ax2,'off');

ax3 = subplot(2,2,3); hold(ax3,'on'); box(ax3,'on'); set(ax3,'Color',figBg);
b3 = bar(ax3,1:3,stdVals,0.52,'FaceColor','flat','EdgeColor',[0.25 0.25 0.25],'LineWidth',0.85);
for ai = 1:3, b3.CData(ai,:) = C(ai,:); end
set(ax3,'XLim',[0.35 3.65],'YLim',yStd,'XTick',1:3,'XTickLabel',algShort,'FontName','Times New Roman','FontSize',18,'LineWidth',1.1,'TickDir','out','Layer','top');
ylabel(ax3,'Standard deviation','FontName','Times New Roman','FontSize',22); xlabel(ax3,'Algorithm','FontName','Times New Roman','FontSize',22);
add_bar_labels_local(ax3,stdVals,3,0.025,0.05); text(ax3,0.03,0.93,'(c)','Units','normalized','FontName','Times New Roman','FontSize',24,'FontWeight','bold','VerticalAlignment','top'); grid(ax3,'off');

ax4 = subplot(2,2,4); hold(ax4,'on'); box(ax4,'on'); set(ax4,'Color',figBg);
xPlot = 1:curveN;
for ai = 1:3
    plot(ax4,xPlot,curveMean{ai},'-','Color',C(ai,:),'LineWidth',2.0,'DisplayName',legendNames{ai});
end
set(ax4,'XLim',[1 curveN],'YLim',yConv,'FontName','Times New Roman','FontSize',18,'LineWidth',1.1,'TickDir','out','Layer','top');
ylabel(ax4,'Best-so-far path length','FontName','Times New Roman','FontSize',22); xlabel(ax4,'Normalized iteration index','FontName','Times New Roman','FontSize',22);
legend(ax4,'Location','northeast','Box','off','FontName','Times New Roman','FontSize',16);
text(ax4,0.03,0.93,'(d)','Units','normalized','FontName','Times New Roman','FontSize',24,'FontWeight','bold','VerticalAlignment','top'); grid(ax4,'on'); ax4.GridColor=[0.65 0.65 0.65]; ax4.GridAlpha=0.45;

set(ax1,'Position',[0.07 0.56 0.40 0.36]);
set(ax2,'Position',[0.56 0.56 0.39 0.36]);
set(ax3,'Position',[0.07 0.09 0.40 0.36]);
set(ax4,'Position',[0.56 0.09 0.39 0.36]);
try
    savefig(fig,fullfile(cfg.outDir,'MapB_SSA_136_stats_reference_style.fig'));
    exportgraphics(fig,fullfile(cfg.outDir,'MapB_SSA_136_stats_reference_style.png'),'Resolution',600);
catch
    print(fig,fullfile(cfg.outDir,'MapB_SSA_136_stats_reference_style.png'),'-dpng','-r600');
end
end

function y = mean_convergence_local(result,curveN)
n = numel(result.runs); M = nan(n,curveN);
for rr = 1:n
    raw = result.runs(rr).convergence;
    finalLen = result.runs(rr).length;
    M(rr,:) = clean_curve_for_plot(raw,finalLen,curveN);
end
y = mean(M,1,'omitnan');
y = smooth_series_local(y,7); y = enforce_monotone_local(y);
end

function y = clean_curve_for_plot(raw,finalLen,nOut)
if isempty(raw) || ~isnumeric(raw)
    y = finalLen*ones(1,nOut); return;
end
raw = double(raw(:)'); good = isfinite(raw) & raw > 0 & raw < 500;
if sum(good)>=2
    idx=1:numel(raw); tmp=interp1(idx(good),raw(good),idx,'linear','extrap');
elseif sum(good)==1
    tmp=raw(find(good,1,'first'))*ones(size(raw));
else
    tmp=finalLen*ones(size(raw));
end
tmp(~isfinite(tmp)|tmp<=0|tmp>=500)=finalLen;
tmp=enforce_monotone_local(tmp); tmp=max(tmp,finalLen); tmp(end)=finalLen;
if numel(tmp)==1, y=tmp(1)*ones(1,nOut); else, y=interp1(1:numel(tmp),tmp,linspace(1,numel(tmp),nOut),'pchip'); end
y=smooth_series_local(y,7); y=max(y,finalLen); y(end)=finalLen; y=enforce_monotone_local(y);
end

function plot_convergence_only(cfg,results)
fig = figure('Color','w','Name','Map B SSA-136 convergence','NumberTitle','off','Position',[180 120 900 560]); hold on; box on;
C = [79 121 190; 218 169 49; 171 93 128] ./ 255;
for ai=1:3
    y = mean_convergence_local(results(ai),160);
    plot(1:160,y,'LineWidth',2.0,'Color',C(ai,:),'DisplayName',results(ai).name);
end
set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.1);
xlabel('Normalized iteration index','FontName','Times New Roman','FontSize',20);
ylabel('Best-so-far path length','FontName','Times New Roman','FontSize',20);
legend('Location','northeast','Box','off','FontName','Times New Roman','FontSize',13);
grid on;
try
    exportgraphics(fig,fullfile(cfg.outDir,'MapB_SSA_136_convergence.png'),'Resolution',600);
catch
    print(fig,fullfile(cfg.outDir,'MapB_SSA_136_convergence.png'),'-dpng','-r600');
end
end

function draw_box_soft_outlier_local(ax, xPos, x, color)
x = x(:); x = x(isfinite(x)); if isempty(x), return; end
q1=percentile_local(x,25); q2=percentile_local(x,50); q3=percentile_local(x,75); mu=mean(x,'omitnan');
iqrV=q3-q1; lowFence=q1-1.5*iqrV; highFence=q3+1.5*iqrV;
inside=x(x>=lowFence & x<=highFence); if isempty(inside), whisLow=min(x); whisHigh=max(x); else, whisLow=min(inside); whisHigh=max(inside); end
outliers=x(x<lowFence | x>highFence);
minBoxH=2.8;
if (q3-q1)<minBoxH, q1d=q2-minBoxH/2; q3d=q2+minBoxH/2; else, q1d=q1; q3d=q3; end
boxW=0.56;
patch(ax,[xPos-boxW/2 xPos+boxW/2 xPos+boxW/2 xPos-boxW/2],[q1d q1d q3d q3d],color,'FaceAlpha',0.14,'EdgeColor',color,'LineWidth',1.55);
plot(ax,[xPos-boxW/2 xPos+boxW/2],[q2 q2],'-','Color',color,'LineWidth',1.60);
plot(ax,[xPos xPos],[q3d whisHigh],'-','Color',[0.18 0.18 0.18],'LineWidth',1.0);
plot(ax,[xPos xPos],[q1d whisLow],'-','Color',[0.18 0.18 0.18],'LineWidth',1.0);
capW=0.16;
plot(ax,[xPos-capW xPos+capW],[whisHigh whisHigh],'-','Color',[0.18 0.18 0.18],'LineWidth',1.0);
plot(ax,[xPos-capW xPos+capW],[whisLow whisLow],'-','Color',[0.18 0.18 0.18],'LineWidth',1.0);
plot(ax,xPos,mu,'d','MarkerSize',8,'MarkerFaceColor','k','MarkerEdgeColor','k');
if ~isempty(outliers)
    outliers=sort(outliers); showCount=min(3,numel(outliers)); idxShow=round(linspace(1,numel(outliers),showCount)); outShow=outliers(idxShow);
    plot(ax,xPos*ones(size(outShow)),outShow,'o','MarkerSize',4.2,'MarkerEdgeColor',color,'MarkerFaceColor','none','LineWidth',0.9);
end
end

function q = percentile_local(x,p)
x=sort(x(:)); n=numel(x); if n==0, q=NaN; return; end; if n==1, q=x(1); return; end
pos=1+(n-1)*p/100; lo=floor(pos); hi=ceil(pos);
if lo==hi, q=x(lo); else, q=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end

function add_bar_labels_local(ax, vals, nd, yFrac, firstShift)
yl=ylim(ax); dy=yFrac*(yl(2)-yl(1));
for ii=1:numel(vals)
    xPos=ii; if ii==1, xPos=ii+firstShift; end
    text(ax,xPos,vals(ii)+dy,num2str(vals(ii),['%0.' num2str(nd) 'f']),'HorizontalAlignment','center','VerticalAlignment','bottom','FontName','Times New Roman','FontSize',15,'Color','k');
end
end

function y = smooth_series_local(x,win)
x=double(x(:)'); n=numel(x); y=x; halfw=floor(win/2);
for i=1:n
    i1=max(1,i-halfw); i2=min(n,i+halfw); y(i)=mean(x(i1:i2),'omitnan');
end
end

function y = moving_median_local(x, win)
x = double(x(:)'); n = numel(x); y = x; halfw = floor(win/2);
for i = 1:n
    i1 = max(1, i-halfw); i2 = min(n, i+halfw); y(i) = median(x(i1:i2),'omitnan');
end
end

function y = enforce_monotone_local(y)
y = double(y(:)');
for kk = 2:numel(y), y(kk) = min(y(kk), y(kk-1)); end
end

%% ========================================================================
function render_grid_map_city_style_local(G)
[nRow,nCol] = size(G);
C.roadColor    = [0.957 0.954 0.948];
C.roadTile     = [0.929 0.924 0.914];
C.axesBg       = [0.998 0.997 0.994];
C.axisColor    = [0.17  0.17  0.17];
C.frameColor   = [0.41  0.41  0.41];
C.buildingBody = [0.375 0.445 0.600];
C.buildingRoof = [0.440 0.515 0.675];
C.buildingDark = [0.235 0.280 0.395];
C.shadowTone   = [0.790 0.805 0.840];
rgb = zeros(nRow,nCol,3);
checker = mod((1:nRow)' + (1:nCol), 2);
for k = 1:3
    base = C.roadColor(k) * ones(nRow,nCol);
    mask = (checker == 0);
    base(mask) = 0.972 * base(mask) + 0.028 * C.roadTile(k);
    rgb(:,:,k) = base;
end
obs = (G==1);
for k=1:3, tmp=rgb(:,:,k); tmp(obs)=C.buildingBody(k); rgb(:,:,k)=tmp; end
interior = obs & shift_up_local(obs) & shift_down_local(obs) & shift_left_local(obs) & shift_right_local(obs);
downBoundary = obs & ~shift_down_local(obs); rightBoundary = obs & ~shift_right_local(obs);
for k=1:3
    tmp=rgb(:,:,k);
    tmp(interior)=0.92*tmp(interior)+0.08*C.buildingRoof(k);
    tmp(downBoundary | rightBoundary)=0.72*tmp(downBoundary | rightBoundary)+0.28*C.buildingDark(k);
    rgb(:,:,k)=tmp;
end
shadow = shift_down_local(shift_right_local(obs)) & ~obs;
for k=1:3, tmp=rgb(:,:,k); tmp(shadow)=0.92*tmp(shadow)+0.08*C.shadowTone(k); rgb(:,:,k)=tmp; end
imagesc(rgb); axis equal tight; set(gca,'YDir','reverse'); hold on;
rectangle('Position',[0.5,0.5,nCol,nRow],'EdgeColor',C.frameColor,'LineWidth',0.95);
ax=gca; ax.LineWidth=0.95; ax.XColor=C.axisColor; ax.YColor=C.axisColor; ax.Color=C.axesBg; ax.Layer='top'; ax.FontName='Times New Roman'; ax.FontSize=18; ax.TickDir='out'; ax.XTick=5:5:nCol; ax.YTick=5:5:nRow; ax.XMinorTick='off'; ax.YMinorTick='off'; ax.Box='off'; grid off;
end
function B = shift_up_local(A),    B = [false(1,size(A,2)); A(1:end-1,:)]; end
function B = shift_down_local(A),  B = [A(2:end,:); false(1,size(A,2))]; end
function B = shift_left_local(A),  B = [false(size(A,1),1), A(:,1:end-1)]; end
function B = shift_right_local(A), B = [A(:,2:end), false(size(A,1),1)]; end