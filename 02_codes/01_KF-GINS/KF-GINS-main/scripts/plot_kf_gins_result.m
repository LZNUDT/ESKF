function plot_kf_gins_result(resultDir, truthFile, gnssFile)
%PLOT_KF_GINS_RESULT 绘制 KF-GINS 融合结果。
%
%   plot_kf_gins_result()
%       绘制 ICM20602 的结果。
%
%   plot_kf_gins_result(resultDir)
%       resultDir 为含 KF_GINS_Navresult.nav 的目录。
%       真值 truth.nav 和 GNSS 文件在该目录、上级目录或相邻目录中自动查找。
%
%   plot_kf_gins_result(resultDir, truthFile, gnssFile)
%       显式指定真值和 GNSS 文件。某一项留空则仍自动查找。
%
% 示例：
%   plot_kf_gins_result('D:\ScriptsCode\ESKF\02_codes\01_KF-GINS\KF-GINS-main\dataset\awesome-gins-datasets\ICM20602\result')
%   plot_kf_gins_result('D:\ScriptsCode\ESKF\02_codes\01_KF-GINS\KF-GINS-main\dataset')

    if nargin < 1
        resultDir = '';
    end
    if nargin < 2
        truthFile = '';
    end
    if nargin < 3
        gnssFile = '';
    end

    rootDir = fileparts(fileparts(mfilename('fullpath')));
    files = resolveInputs(rootDir, resultDir, truthFile, gnssFile);
    fprintf('融合结果: %s\n', files.nav);
    fprintf('真值: %s\n', files.truth);
    fprintf('GNSS: %s\n', files.gnss);

    close all;

    nav = readmatrix(files.nav, 'FileType', 'text');
    ref = readmatrix(files.truth, 'FileType', 'text');
    if isempty(files.gnss)
        tObs = (ceil(nav(1, 2)):floor(nav(end, 2)))';
        fprintf('未找到 GNSS 文件，误差曲线按 1 Hz 取样。\n');
    else
        gnss = readmatrix(files.gnss, 'FileType', 'text');
        tObs = gnss(:, 1);
    end

    [tFull, posErr, velErr, attErr] = navError(nav, ref);
    idx = nearestIndex(tFull, tObs);
    plotErrorGroup(tFull(idx), posErr(idx, :), '位置误差', ...
        {'纬度误差', '经度误差', '高程误差'}, 'm');
    plotErrorGroup(tFull(idx), velErr(idx, :), '速度误差', ...
        {'北向速度误差', '东向速度误差', '垂向速度误差'}, 'm/s');
    plotErrorGroup(tFull(idx), attErr(idx, :), '姿态误差', ...
        {'横滚角误差', '俯仰角误差', '航向角误差'}, 'deg');

    if ~isempty(files.std)
        std = readmatrix(files.std, 'FileType', 'text');
        stdIdx = nearestIndex(std(:, 1), tObs);
        tStd = std(stdIdx, 1);
        stdGroups = {
            2,  '位置标准差',           {'北向', '东向', '地向'}, 'm'
            5,  '速度标准差',           {'北向', '东向', '地向'}, 'm/s'
            8,  '姿态标准差',           {'横滚', '俯仰', '航向'}, 'deg'
            11, '陀螺仪零偏标准差',     {'X', 'Y', 'Z'},         'deg/h'
            14, '加速度计零偏标准差',   {'X', 'Y', 'Z'},         'mGal'
            };
        for k = 1:size(stdGroups, 1)
            col0 = stdGroups{k, 1};
            plotStdGroup(tStd, std(stdIdx, col0:col0 + 2), ...
                stdGroups{k, 2}, stdGroups{k, 3}, stdGroups{k, 4});
        end
        if size(std, 2) >= 22
            plotScaleStd(tStd, std(stdIdx, 17:19), std(stdIdx, 20:22));
        end
    end

    plotTrajectory(nav, ref);

    if ~isempty(files.imu)
        plotImuEstimates(readmatrix(files.imu, 'FileType', 'text'));
    end
end

function files = resolveInputs(rootDir, resultDir, truthFile, gnssFile)
    if isempty(resultDir)
        resultDir = fullfile(rootDir, 'dataset', 'awesome-gins-datasets', ...
            'ICM20602', 'result');
    end
    if isfile(resultDir)
        resultDir = fileparts(resultDir);
    end
    if ~isfolder(resultDir)
        error('结果目录不存在: %s', resultDir);
    end

    files.nav = findNamed(resultDir, {'KF_GINS_Navresult.nav'});
    files.std = findNamed(resultDir, {'KF_GINS_STD.txt'});
    files.imu = findNamed(resultDir, {'KF_GINS_IMU_ERR.txt'});
    if isempty(files.nav)
        error('在 %s 中找不到 KF_GINS_Navresult.nav', resultDir);
    end

    if isempty(truthFile)
        files.truth = findNearby(resultDir, {'truth.nav'});
        if isempty(files.truth)
            files.truth = truthFromYaml(resultDir, rootDir);
        end
    else
        files.truth = truthFile;
    end
    if isempty(files.truth) || ~isfile(files.truth)
        error(['找不到真值 truth.nav。请把真值放在结果目录附近，', ...
            '或调用 plot_kf_gins_result(resultDir, truthFile, gnssFile)。']);
    end

    if isempty(gnssFile)
        files.gnss = findNearby(resultDir, {'GNSS-RTK.txt', 'GNSS_RTK.pos'});
        if isempty(files.gnss)
            files.gnss = gnssFromYaml(resultDir, rootDir);
        end
    else
        files.gnss = gnssFile;
    end
    if ~isempty(files.gnss) && ~isfile(files.gnss)
        error('找不到 GNSS 文件: %s', files.gnss);
    end
end

function path = findNamed(folder, names)
    path = '';
    for k = 1:numel(names)
        candidate = fullfile(folder, names{k});
        if isfile(candidate)
            path = candidate;
            return;
        end
    end
end

function path = findNearby(startDir, names)
    path = '';
    parent = fileparts(startDir);
    grand = fileparts(parent);
    dirs = {startDir, parent, grand};
    for d = {parent, grand}
        if ~isfolder(d{1})
            continue;
        end
        listing = dir(d{1});
        for i = 1:numel(listing)
            if listing(i).isdir && listing(i).name(1) ~= '.'
                dirs{end + 1} = fullfile(listing(i).folder, listing(i).name); %#ok<AGROW>
            end
        end
    end
    for i = 1:numel(dirs)
        if ~isfolder(dirs{i})
            continue;
        end
        for k = 1:numel(names)
            candidate = fullfile(dirs{i}, names{k});
            if isfile(candidate)
                path = candidate;
                return;
            end
        end
    end
end

function path = truthFromYaml(resultDir, rootDir)
    path = '';
    imuPath = yamlValue(resultDir, 'imupath', rootDir);
    if isempty(imuPath)
        return;
    end
    candidate = fullfile(fileparts(imuPath), 'truth.nav');
    if isfile(candidate)
        path = candidate;
    end
end

function path = gnssFromYaml(resultDir, rootDir)
    path = yamlValue(resultDir, 'gnsspath', rootDir);
    if ~isempty(path) && ~isfile(path)
        path = '';
    end
end

function path = yamlValue(resultDir, key, rootDir)
    path = '';
    yamls = [dir(fullfile(resultDir, '*.yaml')); ...
        dir(fullfile(fileparts(resultDir), '*.yaml'))];
    token = ['(?:^|\n)', key, ':\s*"?([^"\r\n]+)'];
    for k = 1:numel(yamls)
        text = fileread(fullfile(yamls(k).folder, yamls(k).name));
        match = regexp(text, token, 'tokens', 'once');
        if isempty(match)
            continue;
        end
        raw = strtrim(match{1});
        raw = strrep(raw, '/', filesep);
        prefix = ['.', filesep];
        if startsWith(raw, prefix)
            raw = raw(numel(prefix) + 1:end);
        end
        if isfile(raw)
            path = raw;
        else
            path = fullfile(rootDir, raw);
        end
        if isfile(path)
            return;
        end
        path = '';
    end
end

function [t, posErr, velErr, attErr] = navError(nav, ref)
    nav = uniqueTime(nav);
    ref = uniqueTime(ref);

    t0 = max(nav(1, 2), ref(1, 2));
    t1 = min(nav(end, 2), ref(end, 2));
    nav = nav(nav(:, 2) >= t0 & nav(:, 2) <= t1, :);
    t = nav(:, 2);

    refAtt = ref(:, 9:11);
    navAtt = nav(:, 9:11);
    for k = 1:3
        refAtt(:, k) = rad2deg(unwrap(deg2rad(refAtt(:, k))));
        navAtt(:, k) = rad2deg(unwrap(deg2rad(navAtt(:, k))));
    end

    refPos = interp1(ref(:, 2), ref(:, 3:5), t, 'linear');
    refVel = interp1(ref(:, 2), ref(:, 6:8), t, 'linear');
    refAtt = interp1(ref(:, 2), refAtt, t, 'linear');

    [north, east, height] = blhErrorMeter(nav(:, 3:5), refPos);
    posErr = [north, east, height];
    velErr = nav(:, 6:8) - refVel;
    attErr = navAtt - refAtt;
    attErr = mod(attErr + 180, 360) - 180;
end

function [north, east, height] = blhErrorMeter(est, refPos)
    a = 6378137.0;
    e2 = 0.00669437999013;
    lat = deg2rad(refPos(:, 1));
    dLat = deg2rad(est(:, 1) - refPos(:, 1));
    dLon = deg2rad(est(:, 2) - refPos(:, 2));
    h = refPos(:, 3);
    s = sin(lat);
    tmp = 1 - e2 * s .* s;
    root = sqrt(tmp);
    rm = a * (1 - e2) ./ (root .* tmp);
    rn = a ./ root;
    north = dLat .* (rm + h);
    east = dLon .* (rn + h) .* cos(lat);
    height = est(:, 3) - refPos(:, 3);
end

function idx = nearestIndex(t, tQuery)
    tQuery = tQuery(tQuery >= t(1) & tQuery <= t(end));
    tQuery = unique(tQuery(:), 'stable');
    idx = round(interp1(t, (1:numel(t))', tQuery, 'nearest'));
    idx = idx(~isnan(idx));
end

function plotErrorGroup(t, err, figName, names, unitName)
    figure('Name', figName, 'Color', 'w');
    for k = 1:3
        subplot(3, 1, k);
        plotMarked(t, err(:, k));
        grid on;
        xlabel('周内秒 / s');
        ylabel(['误差 / ', unitName]);
        title(names{k});
    end
    sgtitle(figName);
end

function plotStdGroup(t, std3, figName, names, unitName)
    figure('Name', figName, 'Color', 'w');
    for k = 1:3
        subplot(3, 1, k);
        plotMarked(t, std3(:, k));
        grid on;
        xlabel('周内秒 / s');
        ylabel(['标准差 / ', unitName]);
        title(names{k});
    end
    sgtitle(figName);
end

function plotScaleStd(t, gyrScale, accScale)
    names = {'X', 'Y', 'Z'};
    figure('Name', '比例因子标准差', 'Color', 'w');
    for k = 1:3
        subplot(3, 2, 2 * k - 1);
        plotMarked(t, gyrScale(:, k));
        grid on;
        xlabel('周内秒 / s');
        ylabel('标准差 / ppm');
        title(['陀螺比例因子 ', names{k}]);

        subplot(3, 2, 2 * k);
        plotMarked(t, accScale(:, k));
        grid on;
        xlabel('周内秒 / s');
        ylabel('标准差 / ppm');
        title(['加速度计比例因子 ', names{k}]);
    end
    sgtitle('比例因子标准差');
end

function plotMarked(t, y)
    plot(t, y, 'o', ...
        'MarkerEdgeColor', [0.85 0.25 0.05], ...
        'MarkerFaceColor', 'none', ...
        'MarkerSize', 4.5, ...
        'LineWidth', 0.7, ...
        'LineStyle', 'none');
    hold on;
    plot(t, y, '-.', 'Color', [0.00 0.30 0.65], 'LineWidth', 1.35);
    hold off;
end

function plotTrajectory(nav, ref)
    nav = uniqueTime(nav);
    ref = uniqueTime(ref);
    t0 = max(nav(1, 2), ref(1, 2));
    t1 = min(nav(end, 2), ref(end, 2));
    nav = nav(nav(:, 2) >= t0 & nav(:, 2) <= t1, :);
    ref = ref(ref(:, 2) >= t0 & ref(:, 2) <= t1, :);

    origin = ref(1, 3:5);
    [eastRef, northRef] = blh2local(ref(:, 3:5), origin);
    [eastNav, northNav] = blh2local(nav(:, 3:5), origin);
    upRef = ref(:, 5) - origin(3);
    upNav = nav(:, 5) - origin(3);

    figure('Name', '水平轨迹', 'Color', 'w');
    plot(eastNav, northNav, '.', 'Color', [0.85 0.25 0.05], 'MarkerSize', 8);
    hold on;
    plot(eastNav, northNav, '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 0.7);
    plot(eastRef, northRef, '--', 'Color', [0.00 0.35 0.70], 'LineWidth', 1.2);
    markTruthEnds(eastRef, northRef);
    hold off;
    axis equal;
    grid on;
    xlabel('东向 / m');
    ylabel('北向 / m');
    title('真值轨迹与融合轨迹');
    legend({'滤波点', '融合轨迹（滤波频率）', '真值（真值频率）', ...
        '真值起点', '真值终点'}, 'Location', 'best');

    figure('Name', '三维轨迹', 'Color', 'w');
    plot3(eastNav, northNav, upNav, '.', 'Color', [0.85 0.25 0.05], 'MarkerSize', 8);
    hold on;
    plot3(eastNav, northNav, upNav, '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 0.7);
    plot3(eastRef, northRef, upRef, '--', 'Color', [0.00 0.35 0.70], 'LineWidth', 1.2);
    markTruthEnds(eastRef, northRef, upRef);
    hold off;
    axis equal;
    grid on;
    view(40, 25);
    xlabel('东向 / m');
    ylabel('北向 / m');
    zlabel('天向 / m');
    title('真值轨迹与融合轨迹');
    legend({'滤波点', '融合轨迹（滤波频率）', '真值（真值频率）', ...
        '真值起点', '真值终点'}, 'Location', 'best');
end

function markTruthEnds(east, north, up)
    startColor = [0.10 0.65 0.20];
    endColor = [0.55 0.10 0.65];
    startEdge = [0.00 0.35 0.08];
    endEdge = [0.30 0.00 0.40];
    if nargin < 3
        plot(east(1), north(1), 'p', 'MarkerSize', 16, ...
            'MarkerFaceColor', startColor, 'MarkerEdgeColor', startEdge, 'LineWidth', 1.0);
        plot(east(end), north(end), 'p', 'MarkerSize', 16, ...
            'MarkerFaceColor', endColor, 'MarkerEdgeColor', endEdge, 'LineWidth', 1.0);
        return;
    end
    plot3(east(1), north(1), up(1), 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', startColor, 'MarkerEdgeColor', startEdge, 'LineWidth', 1.0);
    plot3(east(end), north(end), up(end), 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', endColor, 'MarkerEdgeColor', endEdge, 'LineWidth', 1.0);
end

function plotImuEstimates(imuErr)
    plotErrGroup(imuErr(:, 1), imuErr(:, 2:4), '陀螺仪零偏估计', {'X', 'Y', 'Z'}, 'deg/h');
    plotErrGroup(imuErr(:, 1), imuErr(:, 5:7), '加速度计零偏估计', {'X', 'Y', 'Z'}, 'mGal');
    plotErrGroup(imuErr(:, 1), imuErr(:, 8:10), '陀螺仪比例因子估计', {'X', 'Y', 'Z'}, 'ppm');
    plotErrGroup(imuErr(:, 1), imuErr(:, 11:13), '加速度计比例因子估计', {'X', 'Y', 'Z'}, 'ppm');
end

function plotErrGroup(t, err3, figName, names, unitName)
    figure('Name', figName, 'Color', 'w');
    for k = 1:3
        subplot(3, 1, k);
        plot(t, err3(:, k), '-', 'Color', [0.00 0.35 0.70], 'LineWidth', 1.0);
        grid on;
        xlabel('周内秒 / s');
        ylabel(['估计值 / ', unitName]);
        title(names{k});
    end
    sgtitle(figName);
end

function [east, north] = blh2local(blh, origin)
    a = 6378137.0;
    e2 = 0.00669437999013;
    lat = deg2rad(origin(1));
    dLat = deg2rad(blh(:, 1) - origin(1));
    dLon = deg2rad(blh(:, 2) - origin(2));
    s = sin(lat);
    tmp = 1 - e2 * s * s;
    root = sqrt(tmp);
    rm = a * (1 - e2) / (root * tmp);
    rn = a / root;
    north = dLat .* (rm + origin(3));
    east = dLon .* (rn + origin(3)) .* cos(lat);
end

function data = uniqueTime(data)
    [~, keep] = unique(data(:, 2), 'stable');
    data = data(keep, :);
end
