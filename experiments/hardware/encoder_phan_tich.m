function R = encoder_phan_tich(file, varargin)
%ENCODER_PHAN_TICH  Tinh thong so encoder tu du lieu logic analyzer (file CSV).
%   R = encoder_phan_tich('capture.csv')
%   R = encoder_phan_tich('capture.csv', 'xung_moi_vong', 4000, 'ti_so', 10)
%
%   Doc duoc CSV xuat tu PulseView/sigrok (moi dong 1 mau, tan so lay mau ghi o dong ";")
%   va tu Saleae Logic 2 (cot "Time [s]", moi dong 1 lan doi muc).
%   Kenh mac dinh theo huong dan noi day (so thu tu kenh tren logic analyzer, bat dau tu 0):
%     'A',0  'B',1  'Z',2  'PUL',3  'MARK',4   (dat = [] neu khong noi kenh do)
%   Tuy chon khac:
%     'fs'            [Hz] tan so lay mau neu file khong co cot thoi gian va khong ghi samplerate
%     'xung_moi_vong' so xung STEP / 1 vong truc dong co (cai dat driver) -> tinh so vong da quay
%     'so_vong'       so vong truc dong co da quay (neu khong co kenh Z va PUL)
%     'ti_so'         ti so hop so (1 = khong co hop so)
%     've'            true -> ve dang song dau tien
%
%   Ket qua R: PPR (xung kenh A / vong), CPR (dem x4 / vong), do phan giai, chieu, lech pha A-B,
%   ti le dem encoder / xung lenh, so dem / vong truc ra hop so.

o = inputParser;
o.addParameter('A', 0); o.addParameter('B', 1); o.addParameter('Z', 2);
o.addParameter('PUL', 3); o.addParameter('MARK', 4);
o.addParameter('fs', []); o.addParameter('xung_moi_vong', []); o.addParameter('so_vong', []);
o.addParameter('ti_so', 1); o.addParameter('ve', true);
o.addParameter('loc_gai', 20e-6);   % [s] bo xung/gai ngan hon muc nay (nhieu); 0 = khong loc
o.parse(varargin{:}); o = o.Results;

% ---------- doc file ----------
txt = splitlines(string(fileread(file)));
cm  = startsWith(strtrim(txt), [";", "#"]);
fs  = o.fs;
m = regexp(join(txt(cm), " "), 'Samplerate:\s*([\d.]+)\s*([kMG]?)Hz', 'tokens', 'once');
if isempty(fs) && ~isempty(m)
    fs = str2double(m{1}) * 10^(3*find(["" "k" "M" "G"] == m{2}, 1) - 3);
end
body = txt(~cm & strlength(strtrim(txt)) > 0);
hdr  = strtrim(split(body(1), ","));
num  = str2double(split(body(2:end), ","));
if all(isnan(str2double(hdr))), data = num; else, data = str2double(split(body, ",")); hdr = "D" + (0:numel(hdr)-1)'; end
iT = find(contains(lower(hdr), "time"), 1);
if isempty(iT)
    assert(~isempty(fs), 'File khong co cot thoi gian: truyen ''fs'', tan so lay mau');
    t = (0:size(data,1)-1)' / fs; kenh = data;
else
    t = data(:, iT); kenh = data(:, setdiff(1:size(data,2), iT));
end
lay = @(c) laykenh(kenh, c);
A = lay(o.A); B = lay(o.B); Z = lay(o.Z); PUL = lay(o.PUL); MARK = lay(o.MARK);
assert(~isempty(A) && ~isempty(B), 'Can it nhat kenh A va B');
assert(any(diff(A)) && any(diff(B)), 'Kenh A hoac B khong co tin hieu: kiem tra lai so kenh (''A'', ''B'')');
if o.loc_gai > 0                                   % loc gai nhieu (dây do dai, gan day pha / nguon 48 V)
    nGai = 0;
    [A, g] = locgai(t, A, o.loc_gai); nGai = nGai + g;
    [B, g] = locgai(t, B, o.loc_gai); nGai = nGai + g;
    if ~isempty(Z),   [Z, g]   = locgai(t, Z, o.loc_gai);   nGai = nGai + g; end
    if ~isempty(PUL), PUL = locgai(t, PUL, min(o.loc_gai, 1e-6)); end   % xung lenh co the rat hep
    R.gai_da_loc = nGai;
end

% ---------- cua so do: luc MARK = 1 (them 50 ms sau cung cho servo on dinh) ----------
w = true(size(t));
if ~isempty(MARK) && any(MARK)
    i1 = find(MARK, 1); i2 = find(MARK, 1, 'last');
    w = t >= t(i1) & t <= t(i2) + 0.05;
elseif ~isempty(PUL) && any(diff(PUL))
    % khong co MARK: cua so tu xung PUL dau tien den xung cuoi cung (+50 ms cho servo on dinh)
    % -> bo qua xung encoder nhieu khi dong co dung yen (driver vong kin dao dong +-1 dem)
    d = find(diff(PUL) ~= 0);
    w = t >= t(d(1)) - 1e-3 & t <= t(d(end) + 1) + 0.05;
end
len  = @(x) find(w & [false; diff(x) > 0]);      % chi so suon len trong cua so
xuo  = @(x) find(w & [false; diff(x) < 0]);
aL = len(A); aX = xuo(A); bL = len(B); bX = xuo(B);
R.canh_A_len = numel(aL);
R.dem_x4     = numel(aL) + numel(aX) + numel(bL) + numel(bX);
% giai ma vuong pha co dau: rung +-1 dem khi dung yen tu trieu tieu
map = [0 1 3 2];                                  % (A,B) = 00 01 10 11 -> thu tu 0 1 3 2
q = map(2*A(w) + B(w) + 1); dq = mod(diff(q(:)), 4);
R.dem_x4_co_dau = sum(dq == 1) - sum(dq == 3);
R.loi_nhay_coc  = sum(dq == 2);                   % 2 kenh doi cung luc = mat dem
R.xung_PUL   = numel(len(PUL));

% ---------- chieu quay, lech pha, do rong xung ----------
bTaiA = B(aL - 1);                                % muc B ngay truoc suon len A
R.A_di_truoc_B = mean(bTaiA == 0) > 0.5;
TA = diff(t(aL));
if numel(aL) > 2
    dAB = arrayfun(@(k) t(bL(find(bL > aL(k), 1))) - t(aL(k)), 1:numel(aL)-1, 'UniformOutput', false);
    ok  = ~cellfun(@isempty, dAB);
    lech = cell2mat(dAB(ok))' ./ TA(ok);
    R.lech_pha_do = mod(median(lech) * 360 + 180, 360) - 180;     % +90: A truoc B, -90: B truoc A
    cao = arrayfun(@(k) t(aX(find(aX > aL(k), 1))) - t(aL(k)), 1:numel(aL)-1, 'UniformOutput', false);
    okc = ~cellfun(@isempty, cao);
    R.do_rong_xung_A = median(cell2mat(cao(okc))' ./ TA(okc));
else
    R.lech_pha_do = NaN; R.do_rong_xung_A = NaN;
end

% ---------- PPR: uu tien kenh Z (chinh xac, khong phu thuoc cai dat driver) ----------
R.nguon = "";
zL = []; if ~isempty(Z), zL = len(Z); end
R.so_xung_Z = numel(zL);
if numel(zL) >= 2
    giua = arrayfun(@(k) nnz(aL > zL(k) & aL <= zL(k+1)), 1:numel(zL)-1);
    R.PPR = median(giua); R.nguon = sprintf('kenh Z (%d vong day du, A/vong = %s)', numel(giua), mat2str(giua));
    R.so_vong = R.canh_A_len / R.PPR;
else
    if ~isempty(o.so_vong), R.so_vong = o.so_vong; R.nguon = "so vong nhap tay";
    elseif ~isempty(o.xung_moi_vong) && R.xung_PUL > 0
        R.so_vong = R.xung_PUL / o.xung_moi_vong; R.nguon = sprintf('%d xung PUL / %d xung moi vong', R.xung_PUL, o.xung_moi_vong);
    else
        error('Khong co kenh Z: truyen ''xung_moi_vong'' (cai dat driver) hoac ''so_vong''');
    end
    R.PPR = abs(R.dem_x4_co_dau) / 4 / R.so_vong;
end
R.CPR = 4 * R.PPR;
R.do_phan_giai_do = 360 / R.CPR;
R.dem_moi_vong_truc_ra = R.CPR * o.ti_so;
R.do_phan_giai_truc_ra_do = 360 / R.dem_moi_vong_truc_ra;
if R.xung_PUL > 0, R.dem_moi_xung_lenh = R.dem_x4 / R.xung_PUL; else, R.dem_moi_xung_lenh = NaN; end
if ~isempty(o.xung_moi_vong), R.xung_lenh_moi_vong = o.xung_moi_vong; end

% ---------- bao cao ----------
fprintf('\n===== THONG SO ENCODER (%s) =====\n', file);
fprintf('  Dem trong cua so do: %d suon len A, %d dem x4 (co dau %+d, loi %d), %d xung PUL, %d xung Z\n', ...
    R.canh_A_len, R.dem_x4, R.dem_x4_co_dau, R.loi_nhay_coc, R.xung_PUL, R.so_xung_Z);
if isfield(R, 'gai_da_loc'), fprintf('  Da loc %d gai nhieu ngan hon %.0f us\n', R.gai_da_loc, o.loc_gai*1e6); end
fprintf('  So vong truc dong co: %.4f  (theo %s)\n', R.so_vong, R.nguon);
fprintf('  PPR (so vach / vong)      : %.2f\n', R.PPR);
fprintf('  CPR (dem x4 / vong)       : %.2f  -> do phan giai %.5f do\n', R.CPR, R.do_phan_giai_do);
if o.ti_so ~= 1
    fprintf('  Truc ra hop so (%g:1)     : %.1f dem / vong -> %.6f do\n', o.ti_so, R.dem_moi_vong_truc_ra, R.do_phan_giai_truc_ra_do);
end
fprintf('  Lech pha A-B              : %.1f do (ly tuong +-90)   do rong xung A: %.1f %%\n', R.lech_pha_do, 100*R.do_rong_xung_A);
fprintf('  Chieu                     : %s\n', ternary(R.A_di_truoc_B, 'A di truoc B', 'B di truoc A'));
if R.xung_PUL > 0, fprintf('  Dem encoder / 1 xung lenh : %.4f\n', R.dem_moi_xung_lenh); end

if o.ve
    figure('Name','encoder'); k = find(w, 1); k2 = min(numel(t), find(t > t(k) + 12*median(TA), 1));
    if isempty(k2), k2 = numel(t); end
    s = {A, B, Z, PUL}; ten = {'A','B','Z','PUL'};
    for i = 1:4
        subplot(4,1,i); if ~isempty(s{i}), stairs(t(k:k2)*1e3, s{i}(k:k2)); end
        ylim([-0.2 1.2]); ylabel(ten{i}); grid on;
    end
    xlabel('t [ms]');
end
end

function [v, nGai] = locgai(t, v, tmin)
% bo cac doan muc keo dai ngan hon tmin: gan lai bang muc cua doan truoc do
nGai = 0;
while true
    k   = [1; find(diff(v) ~= 0) + 1];                     % dong bat dau moi doan
    dur = [t(k(2:end)); inf] - t(k);
    bad = find(dur(2:end) < tmin, 1) + 1;                   % xu ly tung doan ngan
    if isempty(bad), break; end
    kEnd = numel(v); if bad < numel(k), kEnd = k(bad+1) - 1; end
    v(k(bad):kEnd) = v(k(bad) - 1);
    nGai = nGai + 1;
end
end

function x = laykenh(kenh, c)
if isempty(c) || c + 1 > size(kenh, 2), x = []; else, x = kenh(:, c + 1) > 0.5; end
end
function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
