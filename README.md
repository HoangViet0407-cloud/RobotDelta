# Robot Delta – Mô phỏng & Thực nghiệm

Mô hình Simscape Multibody của robot delta (3 động cơ, 6 thanh truyền, khớp cầu bi mắt trâu M6), kèm bộ công cụ để chạy **cùng một quỹ đạo** trên mô phỏng và robot thật rồi so sánh kết quả.

## Firmware STM32 — Encoder_TXS

Mã nguồn và hướng dẫn cho NUCLEO-F446RE nằm tại
[`firmware/Encoder_TXS`](firmware/Encoder_TXS/README.md): TIM1 đọc encoder,
TIM2 phát STEP/DIR một trục, USART2 nhận lệnh qua USB ST-LINK. Dự án dùng
C++17, tổ chức App/Libs và có snapshot encoder không dùng PRIMASK.
Xem README của firmware để import vào CubeIDE, build và đấu dây.
Hiện đang điều tra hiện tượng quay vượt lệnh trên bộ thử TXS/CZ880;
chưa tích hợp NMPC hoặc xác nhận vận hành robot thật.

## Bắt đầu nhanh

1. Trong MATLAB, mở project: nhấp đúp `RobotDelta.prj`, hoặc chạy `openProject('<thư mục dự án>')`. Thư mục dự án đặt ở đâu cũng được.
   Project sẽ tự thêm đường dẫn, đưa cache vào `work/`, kiểm tra MATLAB/toolbox và tạo cài đặt riêng cho máy. Nếu mở thẳng file `.slx`, model cũng tự thêm đường dẫn.
   Máy mới: xem mục [Làm việc nhóm trên nhiều máy](#làm-việc-nhóm-trên-nhiều-máy).
2. **Mô phỏng:** mở `RobotDelta_Sim` (có trong mục Shortcuts của project) → Run. Muốn đổi quỹ đạo, sửa `params/RobotDelta_Params.m`.
3. **Thực nghiệm:**
   ```matlab
   exp_precheck                          % E0: kiểm tra offline trước khi chạy robot
   exp_run('E2_DuongTron', 'dry')        % tạo + kiểm tra quỹ đạo khớp, xuất lenh_khop.csv
   exp_run('E2_DuongTron', 'sim')        % chạy trên mô phỏng
   exp_run('E2_DuongTron', 'real')       % chạy trên robot thật (cần firmware, xem bên dưới)
   exp_run('E2_DuongTron', 'sim', 'period', 1)   % đổi tham số riêng của thí nghiệm
   ```

## Cấu trúc thư mục

```
RobotDelta/
├── RobotDelta.prj              mở file này trước
├── models/
│   ├── RobotDelta_Sim.slx      model mô phỏng chính (sinh bởi tools/build_RobotDelta_Sim.m)
│   └── import/                 model import gốc từ SolidWorks + smiData (không sửa tay)
├── cad/
│   ├── export/                 RobotDelta.xml + *.STEP (xuất bằng Simscape Multibody Link)
│   └── solidworks/             Khung.STEP (khung, xuất tay từ cụm lắp; Khung.SLDPRT cũ không dùng)
├── params/                     thông số DÙNG CHUNG cho cả nhóm
│   ├── RobotDelta_Params.m     ← thông số mô phỏng, quỹ đạo, driver, giới hạn an toàn
│   ├── RobotDelta_Real.m       ← hình học robot thật (đã đo: giống CAD → Greal = G), θ_home
│   ├── RobotDelta_Local_template.m  mẫu cài đặt riêng từng máy
│   └── RobotDelta_Geometry.m   hình học đo từ CAD (tự sinh, không sửa)
├── local/                      cài đặt RIÊNG của máy này: cổng COM, người dùng, thư mục dữ liệu (không chia sẻ)
├── lib/                        dùng chung cho sim và robot thật
│   ├── RobotDelta_Traj.m       quỹ đạo TCP (4 chế độ)
│   ├── RobotDelta_IK.m / _FK.m động học ngược / thuận
│   ├── RobotDelta_Metrics.m    góc nghiêng bi mắt trâu, góc tay–thanh truyền
│   └── RobotDelta_GeomIdeal.m  hình học đối xứng từ kích thước đo
├── experiments/
│   ├── exp_precheck.m          E0 – kiểm tra trước thực nghiệm
│   ├── exp_run.m               chạy 1 thí nghiệm: dry | sim | real
│   ├── exp_analyze.m           vẽ, so sánh, tính AP/RP
│   ├── exp_jointtraj.m         quỹ đạo TCP → quỹ đạo khớp + kiểm tra giới hạn
│   ├── exp_simulate.m          chạy model với quỹ đạo tùy ý
│   ├── tests/                  E1_DoChinhXac, E2_DuongTron, E3_GapDat
│   └── hardware/RobotLink.m    giao tiếp PC ↔ vi điều khiển (khung mẫu)
├── tools/
│   ├── build_RobotDelta_Sim.m  tạo lại model mô phỏng từ model import
│   ├── rd_fixpaths.m           bỏ đường dẫn tuyệt đối trong model (sau khi import CAD), ưu tiên file STEP
│   ├── rd_checkgeom.m          kiểm tra file CAD thêm tay có đúng phiên bản cụm lắp (so trọng tâm)
│   ├── rd_package.m            đóng gói dự án thành 1 file .mlproj để gửi máy khác
│   ├── rd_startup.m            tự chạy khi mở project (kiểm tra môi trường, cài đặt máy)
│   └── rd_root, rd_local, rd_checkenv, rd_addpath   hàm tiện ích
├── data/                       kết quả: mỗi lần chạy 1 thư mục, không ghi đè
├── work/                       cache Simulink (xóa thoải mái)
└── .gitignore, .gitattributes  cấu hình sẵn cho Git
```

## Làm việc nhóm trên nhiều máy

Dự án **không chứa đường dẫn tuyệt đối nào**:
- Mọi đường dẫn đều tính từ vị trí của thư mục dự án.
- File CAD được MATLAB tìm theo path của project.

Vì vậy chép thư mục `RobotDelta/` sang ổ khác, máy khác, hay thư mục có dấu cách hoặc tên tiếng Việt thì vẫn chạy được. Điều này đã được kiểm tra bằng cách giải nén vào `May 2 - Nguyễn Văn A` rồi chạy lại mô phỏng, thí nghiệm và build.

| Loại | Ở đâu | Chia sẻ? |
|---|---|---|
| Code, model, CAD, thông số robot | `models/ cad/ params/ lib/ experiments/ tools/` | **Có**, cả nhóm dùng chung |
| Cài đặt riêng của máy: cổng COM, tên người dùng, thư mục dữ liệu | `local/RobotDelta_Local.m` (tự tạo lần đầu mở project) | **Không** |
| Cache Simulink | `work/` | Không (tự sinh lại) |
| Kết quả thí nghiệm | `data/` hoặc thư mục chung `Local.dataDir` | Tùy chọn, xem bên dưới |

### Chia sẻ dự án

- **Cách 1 – Git (nên dùng khi từ 2 người trở lên cùng sửa):** cài Git (hoặc GitHub Desktop), rồi đưa thư mục `RobotDelta/` lên một repo private trên GitHub/GitLab.
  - `.gitignore` đã loại sẵn `local/`, `work/` và `data/`.
  - Mỗi người `git clone` về chỗ bất kỳ, rồi mở `RobotDelta.prj`.
  - Tab Project của MATLAB có sẵn các nút Commit/Push/Pull.
- **Cách 2 – Gửi file:** chạy `rd_package` để tạo `RobotDelta_<ngày>.mlproj` (khoảng 5 MB, không gồm `local/`, `work/`, `data/`). Bên nhận nhấp đúp file trong MATLAB, chọn nơi giải nén, và project sẽ tự mở.
- **Không nên:** nhiều người cùng mở một thư mục dự án trên Google Drive/OneDrive. Rất dễ xung đột file `.slx` và cache.

### Lần đầu mở trên máy mới

1. Mở `RobotDelta.prj`. Project tự kiểm tra MATLAB/toolbox và tạo `local/RobotDelta_Local.m`, `work/`, `data/`.
2. Sửa `local/RobotDelta_Local.m`:
   - `Local.port`: cổng nối vi điều khiển (xem bằng `serialportlist("available")`).
   - `Local.user`: tên người chạy.
   - `Local.dataDir`: thư mục lưu kết quả.
3. Chạy thử: `exp_run('E2_DuongTron','sim')`.

### Chia sẻ kết quả thí nghiệm

- Mỗi lần chạy tạo một thư mục `<thời gian>_<thí nghiệm>_<backend>_<người chạy>`, nên nhiều người ghi chung một chỗ cũng không bị trùng. `run.mat` lưu cả tên người, tên máy và phiên bản MATLAB.
- Muốn dùng chung: trên mọi máy, đặt `Local.dataDir` trỏ tới cùng một thư mục đồng bộ (ví dụ Google Drive).
- Xem lại kết quả của người khác: `exp_analyze('<tên thư mục>')`.

### Quy tắc tránh xung đột

- **Không sửa tay** `models/RobotDelta_Sim.slx` và `params/RobotDelta_Geometry.m`, vì đây là file sinh tự động. Hãy sửa `params/*.m` hoặc `tools/build_RobotDelta_Sim.m` rồi chạy build. File `.slx` là file nhị phân, hai người cùng sửa thì rất khó gộp.
- Cả nhóm dùng **cùng MATLAB R2025b** (kèm Simulink, Simscape, Simscape Multibody). Máy dùng bản cũ hơn không mở được `.slx`; `rd_checkenv` sẽ cảnh báo khi mở project.
- Khi thêm file CAD: đặt vào `cad/export/` hoặc `cad/solidworks/`, và trong model **chỉ ghi tên file**. Có thể chạy `rd_fixpaths('<tên model>')` để tự sửa.
- Nên xuất `Khung` sang STEP như các chi tiết khác. STEP là định dạng trung gian an toàn nhất khi đổi máy hoặc hệ điều hành.
- Linux/macOS phân biệt chữ hoa/thường trong tên file: giữ đúng tên như trong model.

## Quy ước

- **Hệ tọa độ robot:** gốc O ở tâm đế, ngang cao độ trục động cơ, **z hướng lên**. Vị trí nghỉ (home) là (0, 0, −431.4) mm.
- **Góc tay θ:** góc của cánh tay trên so với phương ngang, **dương khi cúi xuống**. Mô phỏng, `lenh_khop.csv` và giao thức với vi điều khiển đều dùng chung quy ước này.
- **Quỹ đạo** (`Traj.mode`): `0` đứng yên · `1` đường tròn · `2` gắp–đặt A↔B · `3` đi qua danh sách điểm.
  Các đoạn khởi động/chuyển tiếp có vận tốc và gia tốc bằng 0 ở hai đầu (không giật, tránh mất bước).

## Kiến trúc thực nghiệm

```
RobotDelta_Traj ──► RobotDelta_IK ──► exp_jointtraj ──► kiểm tra giới hạn ──► lenh_khop.csv
 (tham chiếu TCP)    (G | Greal)       θ(t), ω, xung      (bi mắt trâu, góc,
                                                            kỳ dị, tốc độ)
                                              ├─ dry : dừng ở đây
                                              ├─ sim : RobotDelta_Sim.slx  → góc, mô-men, TCP
                                              └─ real: RobotLink ─USB─► MCU ─► driver ─► 3 động cơ
                                                         ▲── góc đo (đếm bước / encoder) ──┘
exp_analyze ◄── run.mat : so sánh đặt / mô phỏng / thật, AP–RP, báo cáo + hình
```

- Mô phỏng dùng hình học CAD (`G`). Robot thật dùng hình học đối xứng lấy từ kích thước đo (`Greal`).
- Quỹ đạo vi phạm giới hạn sẽ **không được gửi xuống robot thật** (có thể ép gửi bằng `'force', true`, nhưng không nên).

## Quy trình thực nghiệm đề xuất

| Bước | Mục tiêu | Cách làm | Kết quả / tiêu chí |
|---|---|---|---|
| **E0** Kiểm tra offline | Phát hiện sai CAD, vùng làm việc, mô-men cần | `exp_precheck` | Không có cảnh báo nghiêm trọng |
| **B1** Đo & hiệu chuẩn | Đưa kích thước thật vào mô hình | Đo R, r, La, Lr, φ (thước cặp); đo θ_home khi chạm công tắc (thước đo góc); nhập vào `RobotDelta_Real.m`; khai báo `Lim.torque`, `Lim.omega` theo datasheet | Chạy lại `exp_precheck` |
| **B2** Chạy khô + mô phỏng | Duyệt quỹ đạo trước khi chạy máy | `exp_run(..., 'dry')` rồi `'sim'` | Đạt giới hạn, mô-men < mô-men cho phép |
| **E1** Độ chính xác & lặp lại | AP, RP (theo ISO 9283) | `E1_DoChinhXac`: 5 điểm × 10 chu kỳ, bút chấm lên giấy; đo vết, điền vào `do_tay.csv` → `exp_analyze(thư_mục)` | Gợi ý: RP ≤ 1 mm |
| **E2** Đường tròn theo tốc độ | Sai số đường đi, ngưỡng mất bước | `E2_DuongTron` với `'period'` = 4, 2, 1 s; so vết vẽ với đường tròn đặt | Vết không trôi qua các vòng |
| **E3** Gắp–đặt | Chu kỳ, tỉ lệ thành công, mô-men | `E3_GapDat`, giảm dần `'period'` | Về đúng home sau N chu kỳ |

Nên chạy chậm trước (`period` dài, không tải), luôn có **nút dừng khẩn** cắt nguồn driver, và cài thêm giới hạn góc mềm trong firmware.

### Dữ liệu mỗi lần chạy (`<thư mục dữ liệu>/<yyyyMMdd_HHmmss>_<thí nghiệm>_<backend>_<người chạy>/`)

- `run.mat`: toàn bộ cấu hình, quỹ đạo, dữ liệu đo và thông tin người/máy chạy.
- `lenh_khop.csv`: bảng (t, θ1, θ2, θ3, io) gửi cho firmware.
- `do_tay.csv`: chỉ có ở E1, là bảng để điền số đo bằng tay.
- `quy_dao.png`, `khop.png`, `kiem_tra.png`, `bao_cao.txt`.

## Giao thức PC ↔ vi điều khiển (`RobotLink`)

ASCII, mỗi lệnh 1 dòng, vi điều khiển trả lời đúng 1 dòng. Firmware cần viết theo giao thức này (hoặc sửa `RobotLink.m` cho khớp firmware có sẵn).

| PC → MCU | Ý nghĩa | MCU → PC |
|---|---|---|
| `E 1` / `E 0` | bật / tắt driver | `ok` |
| `H` | chạy về công tắc home | `ok` (khi xong) |
| `T <ms>` | chu kỳ nội suy | `ok` |
| `P <θ1> <θ2> <θ3> <io>` | thêm 1 điểm vào bộ đệm | `S <ms> <θ1> <θ2> <θ3> <chỗ trống>` |
| `?` | hỏi trạng thái | `S <ms> <θ1> <θ2> <θ3> <chỗ trống>` |

Firmware cần có:
- Bộ đệm ≥ 32 điểm, mỗi `T` ms lấy ra 1 điểm và nội suy.
- Đổi góc sang bước: `bước = (θ − θ_home) · bước/độ · chiều`.
- Homing bằng công tắc hành trình, và cổng IO cho bơm hút.

Giao thức đã được thử với một vi điều khiển giả lập (có cả trường hợp bộ đệm đầy), **nhưng chưa thử với phần cứng thật**.

## Kết quả E0 hiện tại (2026-09-30, `data/20260930_162108_precheck_nguye`)

| Hạng mục | Kết quả | Ý nghĩa |
|---|---|---|
| Vị trí 3 trục động cơ | R = 32.3 / 48.8 / 81.4 mm tính từ tâm | **Không đối xứng**, đã đo xác nhận robot thật giống CAD → robot thật dùng hình học CAD (`Real.dung_CAD = true`), IK tính riêng từng chân |
| Hình bình hành | CAD: 2 bi trên tay cách 66 mm, trên bàn động 68 mm. Build script đã dời bi trên trục ren để cả hai bằng 68 mm (`W_ARM`); IK giờ khớp tuyệt đối với Simscape | Trên robot thật: chỉnh đai ốc để 2 bi trên trục ren cách 68 mm; sửa luôn trong SolidWorks |
| Thanh truyền | Dài 374.2 mm tâm–tâm (tên file ghi `450mm` là tên cũ); vật liệu **nhôm** (`VatLieu.rho_thanh = 2700`): 45.3 g/thanh, 6 thanh 0.27 kg (thép: 0.79 kg) | Khối lượng tính từ hình học STEP |
| Vùng làm việc an toàn | Tròn R ≈ **82 mm** quanh tâm, z = −382 … −482 mm | Bị giới hạn bởi **góc nghiêng bi mắt trâu** (giả sử ≤ 13°, cần tra datasheet) |
| Động cơ thật (đo 2026-09-30, `solieu_encoder/`) | Động cơ bước lai có encoder, driver HDB CZ880 đặt **800 xung/vòng**, hộp số **1:10**, encoder **1000 vạch (4000 đếm/vòng động cơ, 40 000 đếm/vòng cánh tay)**, lệch pha A–B ±86–90°, xung rộng 51% | Phân tích bằng `experiments/hardware/encoder_phan_tich.m` (lọc gai nhiễu 1 µs, giải mã vuông pha); 3 lần đo đều đúng 4000 đếm/vòng |
| Độ phân giải | **0.20–0.22 mm** TCP mỗi xung lệnh (0.045° cánh tay); encoder đọc được **0.04 mm** (0.009°) | Đủ cho gắp–đặt |
| Mô-men cần (mô phỏng, thanh nhôm) | Tại cánh tay: giữ tĩnh 0.45–0.51 N·m; tròn R60/2 s: 0.60 N·m; gắp–đặt 2 s: 0.67 N·m. **Quy về trục động cơ (÷10): 0.05–0.07 N·m** (chưa tính hiệu suất hộp số) | Rất thấp so với mô-men của động cơ bước lai cỡ 57–60 mm; cần khai báo `Lim.torque` theo datasheet |
| Tần số xung | Tối đa khoảng 1.3 kHz (gắp–đặt 2 s/chu kỳ) | Bo MKS DLC32 và các vi điều khiển thông dụng đều đáp ứng |
| Lệnh động cơ trong mô phỏng | Khối IK xuất θ, θ̇, θ̈ tính chính xác (`RobotDelta_IKd`, sai phân trung tâm h = 10⁻⁴ s); SPS ở chế độ 3 cổng (*Provide signal and derivatives*) | Sai số bám TCP 0.000 mm (chế độ lọc 5 ms cũ: 1.89 mm do trễ); mô-men không đổi |
| Khung | **Đã sửa (2026-09-28):** `Khung.SLDPRT` cũ là phiên bản khác (trọng tâm cao hơn 68.2 mm) nên tấm đế trông như tụt xuống. Model giờ dùng `Khung.STEP` xuất từ cụm lắp, `rd_checkgeom` báo lệch 0.1 mm | Không cần làm gì thêm; có thể xóa `Khung.SLDPRT` cũ. Khung gắn cứng với đất nên các kết quả động lực học không đổi |

## Bộ điều khiển dự báo NMPC (`nmpc/`)

Phương án A: NMPC chạy trên máy tính (sau này là Raspberry Pi 4) và xuất góc/vận tốc đặt cho driver bước vòng kín. Mô hình động lực học được dùng để dự báo mô-men và ràng buộc mô-men.

| Thành phần | Chức năng |
|---|---|
| `lib/RobotDelta_Dyn.m` | Động lực học ngược rút gọn (Codourey): τ = I_b·q̈ − g·m_r·cos q + Jᵀ·m_p·(p̈ − g). Tuyến tính theo 3 tham số |
| `tools/rd_identdyn.m` | Nhận dạng `[I_b m_r m_p]` từ mô-men Simscape, ghi vào `params/RobotDelta_DynParams.m`. Sai số kiểm chứng 2.1 % |
| `nmpc/rd_dyn_sym.m` | Cùng mô hình, viết để chạy được với CasADi |
| `nmpc/RobotDelta_NMPC_build.m` | Tạo bài toán OCP bằng acados và sinh mã C vào `work/nmpc_c` |
| `nmpc/nmpc_sim.m` | Mô phỏng vòng kín trong MATLAB: sai số TCP, mô-men, thời gian giải |
| `tools/rd_acados.m` | Nạp acados và CasADi. Đường dẫn đặt trong `local/RobotDelta_Local.m` |
| `tools/build_RobotDelta_NMPC.m` | Tạo `models/RobotDelta_NMPC.slx` từ `RobotDelta_Sim.slx`: thay khối IK bằng subsystem NMPC (ChuanBi → S-function acados → NoiSuy) |
| `nmpc/nmpc_simscape.m` | Chạy NMPC với robot Simscape đầy đủ, so với IK truyền thẳng. Mô-men lấy từ Simscape |

**Subsystem NMPC** (`RobotDelta_NMPC.slx`):
- **ChuanBi** (rời rạc, Ts): tính tham chiếu N bước từ IK của quỹ đạo. Trạng thái gửi cho bộ giải là x0 = trạng thái lệnh + [d; 0], trong đó d = góc encoder − góc lệnh (lấy cùng thời điểm qua một khối Memory). Nếu driver trễ hoặc mất bước thì d ≠ 0 và NMPC tự lập lại quỹ đạo.
- **acados_NMPC**: S-function `work/acados_solver_sfunction_robotdelta`, tạo bằng `RobotDelta_NMPC_build(true)`.
- **NoiSuy** (liên tục): lệnh cho driver trong một chu kỳ, q = q_c + q̇_c·τ + u·τ²/2. Trên robot thật, vi điều khiển phát xung STEP theo đúng biểu thức này.

Lần đầu trên máy mới: `RobotDelta_NMPC_build(true); build_RobotDelta_NMPC; nmpc_simscape()`.

Bài toán: trạng thái x = [q; q̇], đầu vào u = q̈, mô hình ẋ = [q̇; u].
- Chi phí bám theo q_r, q̇_r, q̈_r lấy từ IK của quỹ đạo.
- Ràng buộc: góc, vận tốc, gia tốc, và |τ| ≤ `Nmpc.tau_max` (ràng buộc mềm).
- Bộ giải: SQP-RTI + HPIPM, Ts = 10 ms, N = 20. Các tham số nằm trong `RobotDelta_Params.m` → `Nmpc`.

**Cài đặt, mỗi máy một lần.** Cần MATLAB, Visual Studio 2022 (trình biên dịch C) và git.
```powershell
mkdir C:\Matlab2025\tools_ext; cd C:\Matlab2025\tools_ext     # thư mục cạnh dự án
# CasADi 3.7.2 (acados chưa kiểm thử với bản 3.8)
Invoke-WebRequest https://github.com/casadi/casadi/releases/download/3.7.2/casadi-3.7.2-windows64-matlab2018b.zip -OutFile casadi.zip
Expand-Archive casadi.zip casadi
git clone --recurse-submodules https://github.com/acados/acados.git
cd acados; mkdir build; cd build
$cm = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
& $cm -G "Visual Studio 17 2022" -A x64 -DACADOS_INSTALL_DIR="C:/Matlab2025/tools_ext/acados" -DBUILD_SHARED_LIBS=OFF -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC ..
& $cm --build . -j 8 --target INSTALL --config Release
```
Nếu đặt ở chỗ khác, khai báo `Local.acadosDir` và `Local.casadiDir`. Sau đó chạy trong MATLAB: `R = nmpc_sim();`

Kết quả mô phỏng (2026-10-11, máy VO1HV, driver lý tưởng):

| Trường hợp | Sai số TCP | max \|τ\| | Thời gian giải trung bình / lớn nhất |
|---|---|---|---|
| Đường tròn R60, 2 s/vòng | < 0.001 mm | 0.60 N·m | 0.44 / 1.3 ms |
| Gắp–đặt 0.6 s/chu kỳ, τ_max = 2 | RMS 0.017 mm | 1.33 N·m | 0.44 / 0.9 ms |
| Gắp–đặt 0.6 s/chu kỳ, **τ_max = 0.8** | RMS 22.6 mm (chủ động chậm lại) | **0.82 N·m** (giữ giới hạn) | 1.0 / 2.7 ms |

Trên robot Simscape đầy đủ (`nmpc_simscape`, mô-men đo từ Simscape):

| Trường hợp | IK truyền thẳng: TCP / max\|τ\| | NMPC: TCP RMS / max | NMPC max\|τ\| | Thời gian giải TB / max |
|---|---|---|---|---|
| Đường tròn R60, 2 s/vòng | 0 mm / 0.60 N·m | < 0.001 / < 0.001 mm | 0.60 N·m | 0.41 / 1.0 ms |
| Gắp–đặt 0.6 s/chu kỳ | 0 mm / 1.25 N·m | 0.017 / 0.068 mm | 1.26 N·m | 0.43 / 1.1 ms |
| Gắp–đặt 0.6 s/chu kỳ, τ_max = 0.8 | 0 mm / 1.25 N·m | 23.9 / 60.2 mm | **0.84 N·m** (vượt 5 % do mô hình rút gọn sai 2 % và RTI) | 1.0 / 2.5 ms |

## Khi sửa CAD

1. Trong SolidWorks, xuất lại bằng Simscape Multibody Link vào `cad/export/` (XML + STEP).
2. Trong MATLAB: `cd models/import`, rồi chạy `smimport(fullfile('..','..','cad','export','RobotDelta.xml'))`. Lệnh này ghi đè `RobotDelta.slx` và `RobotDelta_DataFile.m`.
   Sau đó chạy `rd_fixpaths('RobotDelta'); save_system('RobotDelta')` để bỏ đường dẫn tuyệt đối của máy vừa import. Chi tiết nào SolidWorks không xuất được (như `Khung`) sẽ được tự tìm trong `cad/`.
3. Chạy `build_RobotDelta_Sim`. Script tạo lại `RobotDelta_Sim.slx`, `RobotDelta_Geometry.m` và in phần kiểm tra IK.
   Script dựa vào tên block do smimport sinh ra (`Revolute15`, `Part1_1_RIGID`, …). Nếu mate trong SolidWorks thay đổi, cần cập nhật các danh sách tên trong script.

## Thông tin còn thiếu để hoàn thiện phần robot thật

- Vi điều khiển và firmware đang dùng (Arduino / STM32 / ESP32…), driver, có encoder không, tỉ số truyền.
- Động cơ HS57-64, thân dài 76 mm, hộp số 57PX10 (1:10). **Chưa có datasheet đúng mã.** Đang dùng giá trị điển hình của dòng 57 mm dài 76 mm: mô-men giữ 2,0 N·m, quán tính rôto 4,8·10⁻⁵ kg·m² (quy về cánh tay gần bằng I_b), suy ra `Nmpc.tau_max = Lim.torque` ≈ 8,2 N·m. Cần datasheet hoặc đo thực nghiệm: mô-men giữ, đường mô-men–tốc độ, quán tính rôto.
- Datasheet bi mắt trâu M6 (góc nghiêng cho phép) → `Lim.tilt_rodend`.
- Góc tay khi chạm công tắc home → `Real.theta_home` trong `params/RobotDelta_Real.m`.
