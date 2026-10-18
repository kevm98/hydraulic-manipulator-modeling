function [DelayTable, EE_Metrics] = Delay_Metrics(dataset, matFileName)
% DELAY_METRICS  Measure macroscopic delays and EE tracking error
% with sub-sample crossing times (linear interpolation).
%
% Usage (after simulation):
%   out = sim('Mulag_FME_500_with_cylinder');
%   [DelayTable, EE_Metrics] = Delay_Metrics(out.logsout, 'myDelays.mat');
%
% Expects these signals in 'dataset' (Simulink.SimulationData.Dataset):
%   cmd1..cmd4, q1..q4,
%   Act1_Ext..Act4_Ext,
%   Act1_PA/PB .. Act4_PA/PB,
%   e_norm  (end-effector tracking error norm)

    if nargin < 2
        error('Usage: [DelayTable, EE_Metrics] = Delay_Metrics(out.logsout, ''myDelays.mat'');');
    end

    names = dataset.getElementNames;
    nJ = 4;  % four hydraulic joints

    % ----- thresholds for command onset (T0) -----
    cmd_perc    = 0.05;   % 5% of command span
    cmd_abs_min = 0.002;  % at least 0.2% spool

    % ----- preallocate struct array with fixed fields -----
    base = struct('Joint',NaN, ...
                  'T0',NaN,'T1',NaN,'T2',NaN,'T3',NaN, ...
                  'Tau_valve2cyl',NaN,'Tau_valve2press',NaN,'Tau_valve2joint',NaN, ...
                  'Tau_press2cyl',NaN,'Tau_press2joint',NaN);
    delays = repmat(base,nJ,1);

    % ================== PER JOINT LOOP ==================
    for j = 1:nJ
        delays(j).Joint = j;

        % ====== fetch timeseries for this joint ======
        name_cmd = sprintf('cmd%d',     j);
        name_q   = sprintf('q%d',       j);
        name_ext = sprintf('Act%d_Ext', j);
        name_PA  = sprintf('Act%d_PA',  j);
        name_PB  = sprintf('Act%d_PB',  j);

        cmd_ts = get_ts(dataset, names, name_cmd);
        q_ts   = get_ts(dataset, names, name_q);
        ext_ts = get_ts(dataset, names, name_ext);
        PA_ts  = get_ts(dataset, names, name_PA);
        PB_ts  = get_ts(dataset, names, name_PB);

        %% ================== T0: command onset (interp) ==================
        t_cmd = cmd_ts.Time;
        u     = cmd_ts.Data(:);
        u0    = u(1);

        du_span = max(u) - min(u);
        thr_u   = max(cmd_perc * abs(du_span), cmd_abs_min);

        dev_u   = abs(u - u0);
        t0 = get_precise_crossing(t_cmd, dev_u, thr_u);

        if isnan(t0)
            % no command movement -> nothing to do for this joint
            continue;
        end
        delays(j).T0 = t0;

        %% ================== T1: 50% of MAX cylinder stroke (interp) ==================
        t_ext = ext_ts.Time;
        x_c   = ext_ts.Data(:);
        x0    = x_c(1);

        d_x = abs(x_c - x0);          % deviation from initial

        % use only times >= T0, but allow interpolation with the point before
        mask_ext  = t_ext >= t0;
        if any(mask_ext)
            t_ext_win = t_ext(mask_ext);
            d_x_win   = d_x(mask_ext);

            d_x_max = max(d_x_win);
            if d_x_max > 0      % any non-constant motion
                thr_x = 0.5 * d_x_max;      % 50% of max deviation
                t1    = get_precise_crossing(t_ext_win, d_x_win, thr_x);
                if ~isnan(t1)
                    delays(j).T1 = t1;
                    delays(j).Tau_valve2cyl = t1 - t0;
                end
            end
        end

        %% ================== T2: 50% of MAX pressure difference (interp) ==================
        t_PA = PA_ts.Time;
        t_PB = PB_ts.Time;
        if ~isequal(t_PA, t_PB)
            error('Time vectors for %s and %s do not match.', name_PA, name_PB);
        end
        t_dP = t_PA;

        dP  = PA_ts.Data(:) - PB_ts.Data(:);
        dP0 = dP(1);
        d_dP = abs(dP - dP0);

        mask_dP  = t_dP >= t0;
        if any(mask_dP)
            t_dP_win  = t_dP(mask_dP);
            d_dP_win  = d_dP(mask_dP);

            d_dP_max = max(d_dP_win);
            if d_dP_max > 0
                thr_dP = 0.5 * d_dP_max;
                t2     = get_precise_crossing(t_dP_win, d_dP_win, thr_dP);
                if ~isnan(t2)
                    delays(j).T2 = t2;
                    delays(j).Tau_valve2press = t2 - t0;
                    if ~isnan(delays(j).T1)
                        delays(j).Tau_press2cyl = delays(j).T1 - t2;
                    end
                end
            end
        end

        %% ================== T3: 50% of MAX joint angle (interp) ==================
        t_q = q_ts.Time;
        q   = q_ts.Data(:);
        q0  = q(1);

        d_q = abs(q - q0);
        mask_q  = t_q >= t0;
        if any(mask_q)
            t_q_win = t_q(mask_q);
            d_q_win = d_q(mask_q);

            d_q_max = max(d_q_win);
            if d_q_max > 0
                thr_q = 0.5 * d_q_max;
                t3    = get_precise_crossing(t_q_win, d_q_win, thr_q);
                if ~isnan(t3)
                    delays(j).T3 = t3;
                    delays(j).Tau_valve2joint = t3 - t0;
                    if ~isnan(delays(j).T2)
                        delays(j).Tau_press2joint = t3 - delays(j).T2;
                    end
                end
            end
        end
    end

    %% ================== BUILD TABLE ==================
    DelayTable = struct2table(delays);

    %% ================== EE tracking error (e_norm) ==================
    EE_Metrics = compute_ee_metrics(dataset, names, DelayTable);

    %% ================== SAVE & DISPLAY ==================
    save(matFileName, 'DelayTable', 'EE_Metrics');
    fprintf('Delays + EE metrics saved to %s\n', matFileName);
    disp(DelayTable);
    disp(EE_Metrics);
end


% -------- helper: fetch timeseries from dataset --------
function ts = get_ts(dataset, names, sigName)
    idx = find(strcmp(names, sigName), 1, 'first');
    if isempty(idx)
        error('Signal "%s" not found in logsout.', sigName);
    end
    ts = dataset{idx}.Values;
end


% -------- helper: EE metrics from e_norm --------
function EE_Metrics = compute_ee_metrics(dataset, names, DelayTable)
    idx_e = find(strcmp(names, 'e_norm'), 1, 'first');
    if isempty(idx_e)
        warning('e_norm not found in logsout. EE_Metrics will be NaN.');
        EE_Metrics = table(NaN,NaN,NaN,NaN,NaN, ...
            'VariableNames',{'T_window','E_int_e2','E_RMS','E_max','E_mean'});
        return;
    end

    e_ts = dataset{idx_e}.Values;
    t    = e_ts.Time;
    e    = e_ts.Data(:);

    % Start when first valve moves (earliest non-NaN T0)
    if any(~isnan(DelayTable.T0))
        t_start = min(DelayTable.T0(~isnan(DelayTable.T0)));
    else
        t_start = t(1);
    end

    mask  = t >= t_start;
    t_seg = t(mask);
    e_seg = e(mask);

    if numel(t_seg) < 2
        EE_Metrics = table(NaN,NaN,NaN,NaN,NaN, ...
            'VariableNames',{'T_window','E_int_e2','E_RMS','E_max','E_mean'});
        return;
    end

    T_window = t_seg(end) - t_seg(1);
    E_int_e2 = trapz(t_seg, e_seg.^2);
    E_RMS    = sqrt(E_int_e2 / T_window);
    E_max    = max(abs(e_seg));
    E_mean   = trapz(t_seg, abs(e_seg)) / T_window;

    EE_Metrics = table(T_window, E_int_e2, E_RMS, E_max, E_mean, ...
        'VariableNames',{'T_window','E_int_e2','E_RMS','E_max','E_mean'});
end


% -------- helper: Precise Crossing Time (Linear Interpolation) --------
function t_cross = get_precise_crossing(t_vec, data_vec, threshold)
    % Find first index strictly ABOVE or equal to threshold
    idx_after = find(data_vec >= threshold, 1, 'first');

    % No crossing at all
    if isempty(idx_after)
        t_cross = NaN;
        return;
    end

    % Crossing already satisfied at the first sample in the window:
    % we cannot interpolate, so we approximate crossing at this time.
    if idx_after == 1
        t_cross = t_vec(1);
        return;
    end

    % Two points bounding the crossing: (t1, y1) and (t2, y2)
    idx_before = idx_after - 1;
    t1 = t_vec(idx_before);
    y1 = data_vec(idx_before);
    t2 = t_vec(idx_after);
    y2 = data_vec(idx_after);

    % Linear interpolation: t = t1 + (threshold - y1)/(y2 - y1) * (t2 - t1)
    if y2 == y1
        t_cross = t1;  % avoid divide-by-zero if signal is locally flat
    else
        fraction = (threshold - y1) / (y2 - y1);
        t_cross = t1 + fraction * (t2 - t1);
    end
end
