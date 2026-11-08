function assemble_dataset()
% ASSEMBLE_DATASET  Combine data/day_*.mat into data/dataset.mat (agreed layout),
% write a 500-row sample for Part 3 and a short data card.
%
% dataset.mat contains:
%   X (N x 411), Y (N x 1, first-move flow l/s), Yseq (N x Nu),
%   meta (table: day_id, loop_id, k, qmax_eff, scenario_id), feature_names
% Do NOT commit dataset.mat to GitHub (too large). Commit dataset_sample.mat and data_card.txt.

    root   = fileparts(which('config'));
    outDir = fullfile(root, 'data');
    files  = dir(fullfile(outDir, 'day_*.mat'));
    assert(~isempty(files), 'No day_*.mat files found in %s', outDir);

    X = [];  Y = [];  Yseq = [];  cap = [];  loopid = [];  k = [];  day = [];
    nFail = 0;  tLab = [];  seeds = [];
    for i = 1:numel(files)
        R = load(fullfile(outDir, files(i).name));
        X      = [X;      R.X];       %#ok<AGROW>
        Y      = [Y;      R.Y];       %#ok<AGROW>
        Yseq   = [Yseq;   R.Yseq];    %#ok<AGROW>
        cap    = [cap;    R.cap];     %#ok<AGROW>
        loopid = [loopid; R.loop];    %#ok<AGROW>
        k      = [k;      R.k];       %#ok<AGROW>
        day    = [day;    R.day];     %#ok<AGROW>
        nFail  = nFail + R.nFail;
        tLab   = [tLab; R.tLabel];    %#ok<AGROW>
        seeds  = [seeds; R.seed];     %#ok<AGROW>
    end

    meta = table(day, loopid, k, cap, day, ...
                 'VariableNames', {'day_id', 'loop_id', 'k', 'qmax_eff', 'scenario_id'});
    feature_names = make_feature_names();
    assert(size(X, 2) == 411 && numel(feature_names) == 411, 'Feature layout mismatch');
    assert(all(isfinite(X(:))) && all(isfinite(Y)), 'Non-finite values in dataset');

    save(fullfile(outDir, 'dataset.mat'), 'X', 'Y', 'Yseq', 'meta', 'feature_names', '-v7');

    % 500-row sample for Part 3 (fixed seed)
    rs  = RandStream('mt19937ar', 'Seed', 1);
    idx = randperm(rs, size(X, 1), min(500, size(X, 1)));
    Xs = X(idx, :);  Ys = Y(idx);  Yseqs = Yseq(idx, :);  metas = meta(idx, :); %#ok<NASGU>
    save(fullfile(outDir, 'dataset_sample.mat'), 'Xs', 'Ys', 'Yseqs', 'metas', 'feature_names', '-v7');

    % Data card
    fid = fopen(fullfile(outDir, 'data_card.txt'), 'w');
    fprintf(fid, 'Dataset card (generated %s)\n', datestr(now));
    fprintf(fid, 'Samples N            : %d\n', size(X, 1));
    fprintf(fid, 'Days                 : %d (day ids %d..%d)\n', numel(files), min(day), max(day));
    fprintf(fid, 'Base seed(s)         : %s\n', mat2str(unique(seeds(:)).'));
    fprintf(fid, 'Failed solves skipped: %d\n', nFail);
    fprintf(fid, 'Mean label time      : %.2f s per solve\n', mean(tLab));
    fprintf(fid, 'Cap distribution     : cap=%.2f: %.1f%%, cap=%.2f: %.1f%%, other: %.1f%%\n', ...
        1.5, 100*mean(abs(cap - 1.5) < 1e-9), 0.65, 100*mean(abs(cap - 0.65) < 1e-9), ...
        100*mean(abs(cap - 1.5) >= 1e-9 & abs(cap - 0.65) >= 1e-9));
    fprintf(fid, 'Label (Y) range      : %.3f .. %.3f l/s, mean %.3f\n', min(Y), max(Y), mean(Y));
    fprintf(fid, 'Labels at lower bound: %.1f%% (<= 0.2005)\n', 100*mean(Y <= 0.2005));
    fprintf(fid, 'Labels at upper cap  : %.1f%% (within 1e-3 of cap)\n', 100*mean(Y >= cap - 1e-3));
    fprintf(fid, 'Scenarios            : make_training_days (randomised); 2 h Frejo case held out\n');
    fprintf(fid, 'Local solver         : local_mpc_fmincon, upper bound limited to 1.2 l/s\n');
    fclose(fid);

    fprintf('Assembled %d samples from %d days -> %s\n', size(X, 1), numel(files), ...
            fullfile(outDir, 'dataset.mat'));
    type(fullfile(outDir, 'data_card.txt'));
end


function names = make_feature_names()
    names = cell(1, 411);
    names{1} = 'q_prev';  names{2} = 'Tin';  names{3} = 'Tout_field';  names{4} = 'Ta';
    for i = 1:29, names{4 + i}  = sprintf('Tf_%02d', i); end
    for i = 1:29, names{33 + i} = sprintf('Tm_%02d', i); end
    c = 62;
    for n = 1:12
        for i = 1:29
            c = c + 1;
            names{c} = sprintf('DNI_step%02d_seg%02d', n, i);
        end
    end
    names{411} = 'qmax_eff';
end
