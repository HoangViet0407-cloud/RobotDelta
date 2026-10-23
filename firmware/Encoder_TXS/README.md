# Đọc encoder và điều khiển một động cơ với NUCLEO-F446RE

Dự án học STM32, tạo từ bài UART `Led`. TIM1 đếm encoder quadrature ×4;
TIM2 phát STEP cho **một động cơ**; UART qua USB ST-LINK nhận lệnh và gửi kết quả.
**Bản hiện tại 23/10/2026 — DIRECT_ENA v3: nhập số xung, chọn chiều trước, chạy bằng `g`; ENA RUN=LOW.**
STEP/DIR/ENA xuất GPIO 3,3 V, bỏ TXS trên đường điều khiển động cơ.
PB0/A3 đã đổi từ OE sang **ENA+**. Module TXS đọc encoder vẫn giữ nguyên.
Chưa đọc ALM, chưa có homing, tăng/giảm tốc, vòng điều khiển vị trí ngoài hoặc NMPC.
Ngày 23/10, người dùng xác nhận chạy trực tiếp STM32 → driver đã đúng: số đếm
encoder khớp số xung phát và góc quay quan sát thực tế đúng, với ENA RUN=LOW.
Đây là kết quả do người dùng báo; chưa có bảng số liệu của lượt xác nhận này
hoặc phép đo dòng/ngưỡng điện cho đầu vào CZ880 ghi 5–24 V. Chưa kết luận
việc thiếu dây ENA là nguyên nhân duy nhất của hiện tượng quay quá vòng trước đó.
Firmware hiện chỉ hỗ trợ một động cơ và một encoder; phương án ba trục mới
được thảo luận, chưa triển khai trong bản này.
Tên project `Encoder_TXS` được giữ để tiếp tục dùng cấu hình CubeIDE hiện có.

## Lấy dự án từ GitHub và mở trên máy khác

**Phiên bản trong thư mục này: DIRECT_ENA v3, cập nhật ngày 23/10/2026.**
Dự án nằm tại [`firmware/Encoder_TXS`](https://github.com/HoangViet0407-cloud/RobotDelta/tree/main/firmware/Encoder_TXS)
trong repo RobotDelta. Clone bằng Git Bash:

```bash
git clone https://github.com/HoangViet0407-cloud/RobotDelta.git
```

Trong STM32CubeIDE, chọn **File → Import → General → Existing Projects into
Workspace**, trỏ Root directory tới thư mục `RobotDelta/firmware/Encoder_TXS`
vừa clone, rồi chọn project **Encoder_TXS**. Bỏ chọn **Copy projects into workspace**.
Build cấu hình **Debug** để tạo `Debug/Encoder_TXS.elf`, sau đó tạo cấu hình
**Debug As → STM32 C/C++ Application** cho bo của máy đó. Nếu workspace đã có
project cùng tên, mở project đang có hoặc dùng workspace khác.

Repo có `App`, `Core`, các thư viện trong `Drivers`, `.ioc`, linker script và
cấu hình C++17 của CubeIDE. Không lưu `Debug`, `Release`, ELF, log hoặc file
`.launch` riêng của máy. Bản đã biên dịch dùng STM32CubeIDE 2.2.0 với GNU Tools
for STM32 14.3.rel1; firmware package trong CubeMX là STM32Cube FW_F4 V1.28.3.
Không cần Generate Code lại chỉ để build; xem lưu ý giữ thư mục App khi generate
ở mục 7. Các đường dẫn `C:\Matlab2025\...` trong nhật ký phía dưới mô tả máy
phát triển ban đầu, không phải đường dẫn bắt buộc trên máy mới.

## Cập nhật encoder ngày 21/10/2026 — không dùng PRIMASK

- `XX_encoder` và `QuadratureEncoder` không khóa ngắt. TIM1 vẫn đếm phần cứng;
  SysTick là nơi duy nhất cập nhật số đếm tích lũy sau khởi tạo.
- SysTick công bố `count`, thời gian mẫu và A/B/OVF bằng các biến atomic
  32 bit, kèm số phiên bản chẵn/lẻ. `snapshot()` đọc lại nếu SysTick cập nhật
  giữa lúc sao chép. Payload cũng là atomic, không chỉ riêng số phiên bản.
- `std::atomic` được kiểm tra `is_always_lock_free` khi biên dịch; không dùng
  mutex hoặc atomic 64 bit. Các phép tính 64 bit chỉ dùng dữ liệu riêng của ISR.
- `z` gửi yêu cầu; SysTick đặt mốc ở lần lấy mẫu kế tiếp (danh định 1 ms,
  có thể trễ nếu ngắt bị trì hoãn). `loop()` chỉ in `ZERO` và đặt lại cửa sổ
  tốc độ sau khi nhận xác nhận. Trong lúc chờ, chưa nhận arm/move hoặc in mẫu cũ.
- `snapshot()`, `zero()` và `zeroPending()` chỉ gọi từ setup/loop. Không gọi
  `snapshot()` từ ISR ưu tiên cao hơn SysTick: ISR đó có thể ngắt ngang writer
  rồi chờ writer hoàn tất, gây kẹt. Khi có RTOS/nhiều reader phải thiết kế lại
  quy tắc truy cập; hiện tại chỉ có một foreground context.
- Đọc bản tin lấy mẫu hoàn chỉnh gần nhất, thường trễ tối đa khoảng một nhịp
  SysTick trong điều kiện không bị trì hoãn. Count và GPIO được đọc lần lượt
  trong cùng lượt ISR, không phải chốt đồng thời bằng phần cứng.
- PRIMASK vẫn còn ở phần motor/UART và chuyển trạng thái arm. Đây là thay đổi
  phần encoder, không phải loại bỏ mọi vùng tới hạn trong cả chương trình.
- NMPC trong loop vẫn bị ISR tạm ngắt trên CPU một lõi; thay đổi này không tạo
  thực thi song song và chưa đo được ngân sách thời gian NMPC trên MCU.

Đã biên dịch/liên kết Debug và Release bằng toolchain STM32 của dự án,
không có lỗi hoặc cảnh báo trong log build. Debug dùng Flash `text+data` =
30.684 byte; Release = 17.760 byte. RAM `data+bss` = 5.488 byte ở cả hai,
gồm phần stack/heap dự trữ, chưa phải mức sử dụng stack thực tế.
Tại thời điểm build ngày 21/10, chưa nạp/chạy thử bản encoder mới. Kết quả
người dùng xác nhận với bản DIRECT_ENA v3 ngày 23/10 được ghi ở đầu tài liệu.
Log: `C:\Matlab2025\Workspace\build\Encoder_TXS_atomic_Debug_20261021.log`
và `Encoder_TXS_atomic_Release_20261021.log` trong cùng thư mục.
Bản sao trước thay đổi:
`C:\Matlab2025\Workspace\work\encoder_before_atomic_20261021_163216`.
Gõ `?` sau khi nạp để nhận diện banner `Encoder v2: atomic snapshot`.

**Ghi nhận trước bản DIRECT_ENA:** lượt 80 xung ở 50 Hz từng được người dùng báo
`CNT=136069` và quan sát trục quay hơn 3 vòng. Riêng thay đổi đồng bộ encoder
ngày 21/10 chưa chứng minh sửa được lỗi vượt chuyển động. Sau khi chuyển sang
đấu trực tiếp STEP/DIR/ENA với ENA RUN=LOW, người dùng báo số đếm và góc quay
đã đúng; chưa có phép thử đối chứng tách riêng nguyên nhân ENA và đường TXS.

## Chạy bản DIRECT_ENA — 23/10/2026

### Chân điều khiển và giới hạn điện

Trước khi thêm ENA, người dùng xác nhận ENA+ để hở, ENA− nối chung PUL−/DIR−.
Sau khi nối và thử GPIO 3,3 V, người dùng báo động cơ chạy ổn định khi ENA LOW.
Mặc định RUN=LOW áp dụng cho bộ đang thử; chưa có datasheet chính hãng đúng
CZ880 để khái quát kết quả cho các driver khác hoặc xác nhận dòng/ngưỡng điện.

| Nucleo | Ngõ điều khiển CZ880 theo phương án chọn |
|---|---|
| PA0 / Arduino A0 | PUL+ — STEP từ TIM2_CH1 |
| PA4 / Arduino A2 | DIR+ — chiều quay |
| PB0 / Arduino A3 | ENA+ — mức ENA cấu hình bằng firmware |
| GND | PUL−, DIR−, ENA− nối chung |

Đây là **ánh xạ chân**, chưa phải xác nhận tương thích điện. GPIO STM32 phát
0/3,3 V; nhãn CZ880 ghi 5–24 V. Chưa biết dòng ngõ vào và ngưỡng nhận ở 3,3 V,
vì vậy code không bảo đảm GPIO cấp đủ dòng hoặc driver nhận đủ xung. Cần xác nhận
dòng tải trong giới hạn GPIO và dạng sóng có tải trước khi chạy cơ khí; nếu không
đạt, dùng mạch đệm phù hợp. Không dùng trị số dòng tuyệt đối tối đa của MCU làm
dòng thiết kế. Không cấp 5/24/48 V vào các chân GPIO này.

Tháo các đầu ra STEP/DIR/OE của module TXS động cơ và bo MKS khỏi ba đường điều
khiển này. **PB0 không còn nối OE, và tuyệt đối không để PB0 nối cứng 3V3**.
Không dùng kiểu PUL+=5V, PUL−=PA0 với bản GPIO push-pull này. Nguồn 48 V chỉ
cấp riêng cho VDC+/GND nguồn của driver; GND tín hiệu dùng theo bảng trên.
TXS đọc encoder và dây encoder gốc giữ như mục 2; không nối chồng hai nguồn 5 V.

### ENA, arm và stop

`MOTOR_ENA_RUN_HIGH=false` trong `App/systemConfig.h` đặt **RUN=LOW, STOP=HIGH**,
theo kết quả người dùng báo ngày 23/10. Không cần gõ `v` sau mỗi lần reset.
`v` vẫn dùng để chẩn đoán bộ khác: đảo lựa chọn trong RAM khi DISARMED và IDLE;
reset/nạp lại khôi phục giá trị trong config.

| Trạng thái | Mặc định RUN=LOW | Nếu dùng `v` đảo sang RUN=HIGH |
|---|---|---|
| Khởi tạo / `d` / `!` | ENA HIGH | ENA LOW sau khi cấu hình được áp dụng |
| `e`, đang chạy, hoặc vừa chạy hết đoạn xung | ENA LOW | ENA HIGH |
| `v` khi DISARMED | Đổi sang RUN=HIGH; ENA STOP chuyển LOW ngay | Đổi sang RUN=LOW; ENA STOP chuyển HIGH ngay |

`v` không phát STEP nhưng **có đổi mức ENA ngay**, nên có thể thay đổi mô men giữ.
Chỉ đổi cấu hình khi nguồn động lực đã tắt và động cơ tách khỏi cơ cấu.
`d`/`!` ép STEP LOW trước rồi đổi ENA sang mức STOP; có thể cắt xung đang phát.
Sau khi hết một đoạn xung, ENA vẫn ở mức RUN cho tới `d`/`!`.
ENA HIGH được chọn làm STOP; chưa có phép đo riêng xác nhận phản ứng khóa/nhả
mô men ở mức này. Dừng STEP có hiệu lực ở bộ phát; phần mềm không cắt nguồn 48 V
và không thay thế dừng phần cứng.

`ARMED=1` chỉ là quyền phát xung của phần mềm. `ENA_cmd=HIGH/LOW` đọc thanh ghi
đầu ra GPIO (ODR), **không đo điện áp thực ở driver**. `ENA_run` là lựa chọn cực
tính hiện hành. Không suy ra driver đang cho phép chỉ từ các dòng này.

`.ioc` và mã GPIO đã đồng bộ PB0/MOTOR_ENA khởi tạo HIGH, pull-up trước `setup()`.
Nếu đổi cực tính mặc định sau này, cần đổi cả mức khởi tạo GPIO tương ứng.
Pull-up nội chỉ có tác dụng sau cấu hình; phần mềm không bảo đảm trạng thái ENA
trong lúc reset, mất nguồn hoặc trước khi GPIO được khởi tạo.

### Thao tác nạp và vận hành

1. Tắt nguồn động lực khi đấu dây/nạp, dùng một động cơ đã tháo khỏi robot và
   cố định chắc. Dùng ánh xạ chân trên sau khi xác nhận tương thích điện.
2. CubeIDE: chọn project `Encoder_TXS` → **Refresh/F5 → Build Project**.
   Khi cần đọc lại metadata: Close Project → Open Project. Chọn đúng ELF mới
   trong Debug/Run, Resume/F8. Không nạp ELF cũ khi build báo lỗi.
3. Terminal: **115200, 8N1, Flow control None, local echo OFF, local line editing OFF**.
   Gõ `?`, tìm banner **`Encoder_TXS DIRECT_ENA v3`**. Gõ `p` để giảm bản tin,
   `d` rồi `m`: mặc định `ARMED=0 ENA_cmd=HIGH ENA_run=LOW`, `next_pulses=80`.
4. Chọn chiều bằng `f` (thuận) hoặc `b` (ngược). **Từ v3, hai lệnh này chỉ đặt
   DIR, không chạy ngay như v1.** Gõ `n400` rồi Enter để chọn 400 xung. `m` cho
   biết `DIR=+/-`, `next_pulses`, `next_Hz`. Chỉ đổi số xung/chiều khi IDLE.
5. Gõ `z` khi trục đứng yên → đợi `ZERO` → `e` → đợi `ARMED` và driver sẵn sàng
   → **`g`**. Đây là lệnh duy nhất bắt đầu một đoạn STEP. Gõ từng lệnh, đợi phản hồi;
   không dán liền `eg`. Firmware yêu cầu tối thiểu 3 ms sau arm/đổi chiều; đây là thời gian phần mềm chọn,
   chưa phải thời gian đáp ứng ENA được xác nhận cho CZ880.
6. `g` phát đúng số xung đã chọn, phạm vi **1–80.000**, khởi động mặc định 80;
   tốc độ mặc định **100 Hz**. Thời gian danh định là `N/f`, cộng trễ nạp lại ISR.
   `+/-` thay đổi tốc độ lần sau khi IDLE, giới hạn 50–500 Hz. Các lựa chọn giữ
   cho lần `g` tiếp theo; reset trả về 80 xung, 100 Hz, chiều thuận, DISARMED.
7. Đứng yên rồi gõ `r`: độ lớn count dự kiến thay đổi **5 × N** nếu driver nhận đủ
   xung và động cơ bám đúng. Với N=80: **400 count = 36° trục động cơ = 3,6° trục
   hộp số**. Người dùng đã báo số đếm và góc quay khớp khi chạy trực tiếp, nhưng
   chưa cung cấp bảng số liệu của lượt xác nhận. Nếu quay quá góc/rung mạnh như
   trước, dừng bằng `!` và ngắt động lực.
8. `!`/`d` xử lý ngắt STEP và đặt ENA STOP ngay trong callback RX, xóa lệnh chờ;
   muốn phát tiếp phải gõ `e` lại. **`p` chỉ dừng in dữ liệu**.

### Chọn nửa vòng, một vòng hoặc nhiều vòng

Driver đang đặt **800 xung/vòng trục động cơ**, hộp số **1:10**:

| Chuyển động mong muốn | Lệnh đặt số xung (bấm Enter sau lệnh) | Count encoder dự kiến, độ lớn |
|---|---|---:|
| Nửa vòng trục động cơ | `n400` | 2.000 |
| Một vòng trục động cơ | `n800` | 4.000 |
| Hai vòng trục động cơ | `n1600` | 8.000 |
| Nửa vòng trục ra hộp số | `n4000` | 20.000 |
| Một vòng trục ra hộp số | `n8000` | 40.000 |
| Hai vòng trục ra hộp số | `n16000` | 80.000 |

Ví dụ **một vòng trục động cơ chiều ngược**: gõ `p`, `b`, `n800` + Enter, `m`,
`z` → đợi ZERO, `e` → đợi ARMED, rồi `g`. Khi IDLE, gõ `r` để đối chiếu số đếm.
Ở 100 Hz, 800 xung mất khoảng 8 s; 8.000 xung mất khoảng 80 s, cộng trễ ISR.
Chiều thuận/ngược là quy ước phần mềm, chưa phải quy ước cơ khí đã hiệu chuẩn.

`n800` và `n 800` đều được nhận. Trong lúc nhập số, Backspace xóa chữ số cuối;
Esc hủy, `!`/`d` dừng và hủy dòng nhập ngay. Enter có thể là CR, LF hoặc CRLF.
Số 0, số ngoài giới hạn, số âm, số thập phân hoặc dòng lẫn chữ bị từ chối,
giữ giá trị trước. Sau lỗi nhập số quá dài/ký tự sai, Enter hoặc Esc rồi nhập lại.
`n` cần Enter; các phím đơn `f`, `b`, `e`, `g`, `m`, `!` không cần Enter.
Chữ số `1`/`0` trong dòng `n...` là dữ liệu, không bật/tắt LED.
Với local echo OFF, chữ số gõ không hiện ngay; sau Enter xem `next_pulses` trong
bản tin xác nhận để biết số đã nhận.
Nếu đang nhập dở `n...`, các phím khác không được xem là lệnh chạy; kết thúc
bằng Enter hoặc hủy bằng Esc trước. Không có tự lặp; mỗi lần `g` được chấp nhận
chỉ chạy một đoạn và từ chối lệnh chạy mới khi BUSY.

Giới hạn 80.000 xung là giới hạn phần mềm cho bài thử động cơ rời (100 vòng motor,
10 vòng hộp số), **không phải phạm vi an toàn của robot đã lắp**. Chưa có ramp
tốc độ, homing hoặc giới hạn góc theo cơ cấu; không dùng các lệnh quay nhiều vòng
khi động cơ đang gắn cánh tay robot.

### Thay đổi mã nguồn ngày 23/10

- `StepDirMotor`: thay ngõ OE bằng ENA có cực tính cấu hình, trạng thái GPIO
  tách khỏi software arm; dừng STEP trước khi đổi ENA sang STOP.
- `XX_motor`/`XX_comm`/`app`: thêm lệnh `v`, trạng thái `ARMED/ENA_cmd/ENA_run`
  và banner DIRECT_ENA; bỏ thông báo OE dành cho module động cơ.
- `systemConfig.h`: chân ENA và mặc định cực tính. `.ioc`, `main.h`, khởi tạo
  GPIO trong `main.c` đồng bộ nhãn PB0 thành `MOTOR_ENA`; logic vẫn ở App/Libs.
- Cơ chế TIM2 PWM2/OPM và bộ đếm encoder atomic được giữ nguyên.
- v3: ENA RUN=LOW, PB0 khởi tạo HIGH/pull-up; `n` chọn số xung, `f/b` chỉ chọn
  chiều, `g` chạy. Parser số ở `XX_comm` trong foreground, ISR chỉ nhận byte/dừng.
- Parser chặn tràn số nguyên, giữ giá trị trước khi nhập sai; cài đặt motor đổi
  khi IDLE. ENA vẫn ở RUN sau khi hết xung, cho tới `d`/`!`.
- Sao lưu trước sửa: `C:\Matlab2025\Workspace\work\encoder_before_direct_ena_20261023_085149`.
- Sao lưu trước khi đổi mặc định ENA: `C:\Matlab2025\Workspace\work\encoder_before_ena_low_20261023_094909`.
- Sao lưu trước khi thêm số xung thay đổi: `C:\Matlab2025\Workspace\work\encoder_before_variable_pulses_20261023_095259`.
- Người dùng đã báo chạy bản hiện tại đúng số đếm encoder và góc quay thực tế
  khi nối trực tiếp STM32 → driver, ENA RUN=LOW. Chưa có bảng dữ liệu của lượt
  xác nhận, đo dòng tải hoặc kết luận nguyên nhân duy nhất của lỗi trước đây.

### Biên dịch bản DIRECT_ENA v3

Đã biên dịch/liên kết Debug và Release bằng GNU Tools for STM32 14.3.rel1
ngày 23/10/2026, không có diagnostic lỗi/cảnh báo trong hai log. Compiler/linker
chạy ngoài sandbox do lỗi tạo tiến trình 623 đã xác định ở lần build trước.
Việc nạp và chạy trên bo do người dùng thực hiện; kết quả được báo lại ở đầu
tài liệu. Khi chuẩn bị cập nhật GitHub, chỉ đối chiếu mã nguồn, cấu hình và log
build đã có; không chạy lại phép thử hoặc nạp bo.

| Cấu hình | Flash: text + data | RAM: data + bss, gồm vùng stack/heap dự trữ |
|---|---:|---:|
| Debug | 33.584 byte | 5.504 byte |
| Release | 20.040 byte | 5.504 byte |

Các giá trị RAM này chưa phải đo stack khi chạy. ELF mới nằm trong
`Debug/Encoder_TXS.elf` và `Release/Encoder_TXS.elf`. Log trên máy phát triển:

- `C:\Matlab2025\Workspace\build\Encoder_TXS_variable_pulses_Debug_20261023.log`
- `C:\Matlab2025\Workspace\build\Encoder_TXS_variable_pulses_Release_20261023.log`

### Thiết kế phát xung và ngắt

- `StepDirMotor` nhận handle TIM/GPIO qua constructor, không phụ thuộc `dev`,
  cấu hình ứng dụng hay UART. Hiện dùng riêng TIM2/CH1.
- CubeMX cấu hình PWM mode 2, active-high, PSC=79 với clock timer 80 MHz.
  Counter chạy 1 MHz; mặc định ARR=9999, CCR1=5000.
- Thư viện bật **OPM (one-pulse mode)** và **URS**, không chặn update event.
  Mỗi lượt counter có nửa đầu LOW, nửa sau HIGH. Khi tràn, phần cứng trở về
  LOW và tự xóa CEN. Callback update đếm một xung hoàn tất, chỉ khởi động
  lại nếu còn xung trong lệnh.
- Công thức danh định: `f = f_TIM / ((PSC+1)*(ARR+1))`. Khoảng LOW thực tế
  có thêm thời gian chờ ISR; đây chưa phải bộ phát chuỗi xung liên tục dùng DMA.
  ISR bị chậm thì khoảng nghỉ dài hơn, không phát thừa chu kỳ vì OPM tự dừng.
- Mặc định 80 xung/lệnh; `n...` chọn 1–80.000 xung/lệnh; tốc độ 50–500 Hz.
  `+/-` thay đổi 50 Hz khi IDLE. Chưa có ramp; chỉ dùng bài thử một động cơ.
- Giữ tối thiểu 3 ms sau arm và sau lần kết thúc/abort trước khi nhận move.
  Sau đặt DIR, nửa chu kỳ LOW đầu tiên tạo thời gian setup ít nhất 1 ms ở 500 Hz.
  Đây là lựa chọn cho bài thử, chưa phải timing được xác nhận từ datasheet CZ880.
- NVIC group 4: TIM2 ưu tiên 1; USART2 ưu tiên 2; SysTick ưu tiên 3.
  UART RX dùng ngắt + ring buffer 32 byte, TX vẫn blocking trong setup/loop.
  Callback RX xử lý `!`/`d` ngay; UART lỗi hoặc ring đầy cũng khóa motor,
  bỏ lệnh đang chờ và khởi động nhận lại. Lệnh arm cũ bị vô hiệu sau một lần stop.
- Các vùng PRIMASK còn lại ở motor/UART/arm chỉ bảo vệ thao tác trạng thái/thanh
  ghi ngắn, không chứa UART, delay hoặc tính toán dài. Encoder dùng snapshot
  atomic và yêu cầu zero; timer encoder vẫn đếm phần cứng.
- `pulses_done` là **số xung timer hoàn tất**, không chứng minh driver nhận đủ
  hay trục quay đúng. Khi abort có thể cắt xung đang phát; số này không tính
  phần xung bị cắt. Đối chiếu encoder sau khi trục đứng yên.
- TIM2 không bị freeze khi debug: xung đang phát hoàn tất rồi phần cứng dừng,
  các xung còn lại chờ ISR và có thể chạy tiếp khi Resume. Trước debug phải
  gõ `d` hoặc tắt nguồn động lực. TIM1 vẫn freeze như bài encoder trước.
- Mất kết nối PC chưa có watchdog giao tiếp; lệnh đang chạy hoàn tất số xung
  đã nhận rồi dừng. Không có lệnh tự lặp hoặc tự chạy khi khởi động.

Cơ sở OPM/PWM2: [ST AN4776](https://www.st.com/resource/en/application_note/an4776-how-to-use-generalpurpose-timer-peripheral-on-stm32-mcus-stmicroelectronics.pdf).
Nguồn bản sao trước thay đổi:
`C:\Matlab2025\Workspace\work\encoder_before_motor_20261020_230001`.

### Cấu trúc theo guide ngày 17/10/2026

Áp dụng guide “Kinh nghiệm lập trình & tổ chức firmware STM32” do người dùng cung cấp. `Core` giữ mã C do CubeMX tạo; phần ứng dụng dùng C++17:

```text
Encoder_TXS/
├── Core/                       CubeMX: khởi tạo ngoại vi và vector ngắt
├── Drivers/                    ST HAL, CMSIS, BSP
├── App/
│   ├── app.h                   giao diện C: setup(), loop(), appTick1ms()
│   ├── app.cpp                 trình tự khởi động, xử lý lệnh, lịch gửi dữ liệu
│   ├── systemConfig.h          count/vòng, hộp số, chu kỳ gửi, nhãn chân
│   ├── device.h                inline Device dev và khai báo các module
│   ├── XX_encoder.cpp          gắn TIM1, hook SysTick và xác nhận zero trong loop
│   ├── XX_motor.cpp            gắn TIM2, phát đoạn xung, callback timer
│   ├── XX_comm.cpp             UART RX ngắt, lệnh dừng, bản tin góc/tốc độ
│   └── Libs/
│       ├── QuadratureEncoder.h
│       ├── QuadratureEncoder.cpp
│       ├── StepDirMotor.h
│       └── StepDirMotor.cpp
└── Encoder_TXS.ioc
```

`main.c` gọi `setup()` tại `USER CODE BEGIN 2`, sau khi khởi tạo ngoại vi. LED/nút nhấn vẫn do CubeMX/BSP khởi tạo trước vòng lặp. `while (1)` chỉ gọi `loop()`. `app.h` dùng `extern "C"` để nối C với C++; SysTick chuyển tiếp tới `appTick1ms()` trong module encoder.

Project đã có C++ nature, C++17, include `../App`, source `App` và cấu hình G++ linker cho Debug/Release. Thư viện nhận handle HAL/chân qua constructor, không dùng `extern htim1` bên trong `Libs`.

### Kết quả biên dịch cấu trúc C++ ngày 17/10/2026

**Bản ngày 20/10/2026, trước khi đổi đồng bộ encoder:** đã biên dịch và liên kết Debug và Release bằng
STM32CubeIDE 2.2.0 / GNU Tools for STM32 14.3.rel1, **0 lỗi, 0 cảnh báo**.
Debug: Flash `text+data` = 28.588 byte; Release = 17.188 byte.
Cả hai có RAM `data+bss` = 5.472 byte, gồm vùng stack/heap dự trữ;
chưa đo stack thực tế. Chưa nạp/chạy trên bo.
Log: `C:\Matlab2025\Workspace\build\Encoder_TXS_motor_20261020.log`.
Các số liệu ngày 17/10 bên dưới thuộc bản encoder trước khi thêm motor.

Đã biên dịch và liên kết **Debug và Release thành công**, không có diagnostic lỗi/cảnh báo. CubeIDE 2.2.0 sinh Makefile; GNU Tools for STM32 14.3.rel1 biên dịch C bằng GCC, C++17 bằng G++, liên kết bằng G++. Do môi trường sandbox gặp Windows error 623 khi tạo tiến trình con, lần build thành công chạy trực tiếp make của CubeIDE ngoài sandbox với chính Makefile đã sinh.

| Cấu hình | Flash: text + data | RAM: data + bss, gồm vùng dự trữ stack/heap |
|---|---:|---:|
| Debug | 17.752 byte | 5.272 byte |
| Release | 10.960 byte | 5.272 byte |

Chưa nạp/chạy thử phiên bản C++ trên bo; kích thước RAM trên không phải phép đo stack thực tế. Log: `C:\Matlab2025\Workspace\build\Encoder_TXS_cpp_Debug_20261017.log` và `Encoder_TXS_cpp_Release_20261017.log`. Bản trước khi đổi sang C++ được sao lưu tại `C:\Matlab2025\Workspace\work\encoder_before_cpp_guide_20261017`.

### Kết quả biên dịch phiên bản cũ ngày 16/10/2026

Các số liệu dưới đây thuộc phiên bản trước khi chuyển sang `App`; không phải kết quả biên dịch mã hiện tại.

- Đã biên dịch cấu hình **Debug** bằng STM32CubeIDE 2.2.0 / GNU Tools for STM32 14.3.rel1: **0 errors, 0 warnings**.
- File nạp qua CubeIDE: `Debug/Encoder_TXS.elf`.
- Flash theo linker: 17.092 byte (text + data). RAM tĩnh và vùng dự trữ: 5.256 byte (data + bss), gồm 4.096 byte dự trữ stack và 512 byte heap; đây không phải phép đo stack khi chạy.
- Chưa nạp firmware, chưa chạy thử trên bo, chưa xác nhận chất lượng xung qua TXS.
- Log biên dịch trên máy này: `C:\Matlab2025\Workspace\build\Encoder_TXS_build.log`.

## 1. Mở trong STM32CubeIDE

1. Chọn **File → Import → General → Existing Projects into Workspace**.
2. Chọn thư mục chứa README này: `C:\Matlab2025\Cubemx\Encoder_TXS`.
3. Đánh dấu dự án **Encoder_TXS**, bỏ chọn **Copy projects into workspace**, chọn **Finish**.
4. Nếu IDE hỏi lưu file thuộc bài cũ, lưu bài cũ trước rồi chọn đúng dự án Encoder_TXS.
5. Chuột phải Encoder_TXS → **Build Project**. Mã C/C++ và thư viện đã có sẵn; không cần Generate Code trước lần build đầu.
6. Chọn **Debug As → STM32 C/C++ Application**. Kiểm tra mục Project/ELF là Encoder_TXS và `Debug/Encoder_TXS.elf`.
7. Khi debugger dừng tại main, bấm **Resume / F8**. Khi đo phải để chương trình chạy liên tục.

Nếu hiện thông báo **already exist in the workspace** và Finish bị mờ: chọn **Cancel**, tìm Encoder_TXS trong Project Explorer; dự án đã được import.

Sau lần đổi cấu trúc sang C++: kết thúc phiên Debug đang chạy, chọn Encoder_TXS → **Refresh / F5** → **Close Project** → **Open Project** để IDE đọc lại cấu hình. Chọn **Project → Clean**, chỉ chọn Encoder_TXS, rồi **Build Project** để thay object `app.c` cũ bằng các file `.cpp` mới. Không cần import hoặc Convert to C++ lại vì metadata dự án đã được cập nhật.
Nếu menu Debug vẫn chọn bài Led cũ: mở **Run → Debug Configurations**, tạo/chọn cấu hình cho Project **Encoder_TXS** và C/C++ Application **Debug/Encoder_TXS.elf**.

Đường dẫn trong cấu hình dự án dùng vị trí tương đối; có thể chép nguyên thư mục sang máy khác và import lại.

## 2. Đấu dây encoder (module TXS còn sử dụng)

| Thiết bị | Chân nối |
|---|---|
| CZ880 EA+ | TXS B1 |
| CZ880 EB+ | TXS B2 |
| TXS A1 | Nucleo PA8 / D7 / TIM1_CH1 |
| TXS A2 | Nucleo PA9 / D8 / TIM1_CH2 |
| Nucleo 3V3, CN6 chân 4 | TXS VCCA và OE (nối luôn bật cho bài thử) |
| CZ880 VCC encoder, đã đo 5,02 V | TXS VCCB |
| CZ880 EGND + Nucleo GND | TXS GND |

EA− và EB− vẫn nối vào driver. Toàn bộ đường encoder gốc tới driver giữ nguyên; TXS lấy thêm nhánh EA+/EB+.
A1/A2 nói trên là nhãn trên **module TXS**, không phải chân Arduino A1/A2 của Nucleo.
Không nối chồng nguồn VCC encoder với chân 5V của Nucleo. USART2 PA2/PA3 sử dụng đường ST-LINK có sẵn; không cần USB-UART rời.

Đấu dây khi tắt nguồn. Với OE nối thẳng VCCA: cấp USB Nucleo trước, sau đó cấp driver; khi kết thúc tắt driver trước rồi rút USB.
Trước khi nối A1/A2 vào MCU, kiểm tra đầu ra module có mức thấp gần 0 V và mức cao gần 3,3 V tại các vị trí dừng khác nhau.

## 3. Xem dữ liệu trên máy tính

- Mở PuTTY hoặc terminal serial đang dùng, chọn COM của **STLink Virtual COM Port**.
- Cấu hình **115200 baud, 8 data bits, no parity, 1 stop bit, flow control None**.
- Trong PuTTY: Terminal → Local echo = Force off; Local line editing = Force off.
- Chỉ mở một ứng dụng terminal trên COM này.
- Nếu mở terminal sau khi banner đã chạy qua, gõ `?` để xem lại hướng dẫn.

Ví dụ minh họa sau một vòng trục động cơ, chiều được quy ước dương, đã đứng yên:

```text
t=5200 CNT=4000 motor_deg=360.000 gear_deg=36.000 motor_rpm=0.000 DIR=0 AB=10 OVF=0
```

Đây là **ví dụ**, không phải kết quả đã đo bằng firmware trên bo.

| Trường | Ý nghĩa |
|---|---|
| t | Thời gian của mẫu encoder gần nhất kể từ khởi động, ms; quay vòng sau khoảng 49,7 ngày |
| CNT | Số count có dấu, tương đối với lúc khởi động hoặc lần gõ z |
| motor_deg | Góc tích lũy trục động cơ, có thể lớn hơn 360° |
| gear_deg | Góc quy đổi sau hộp số 1:10; không đo trực tiếp độ rơ/biến dạng hộp số |
| motor_rpm | Tốc độ trung bình trong khoảng giữa hai bản tin đo |
| DIR | + / - / 0 theo thay đổi vị trí trong khoảng đo; không tự xác định chiều thuận cơ khí |
| AB | Mức logic PA8/PA9 trong lượt lấy mẫu SysTick đã công bố |
| OVF | 1 khi số đếm phần mềm vượt miền int32; dừng cộng dồn đến khi gõ z. **Không phải cờ phát hiện nhiễu/mất xung** |

## 4. Các phím lệnh

Các phím đơn không cần Enter; riêng `n<số xung>` phải kết thúc bằng Enter:

| Phím | Tác dụng |
|---|---|
| z | Yêu cầu đặt mốc ở nhịp SysTick kế tiếp; chờ dòng ZERO trước e/g. Chỉ dùng khi trục đứng yên |
| s | Bật gửi tự động, khoảng 100 ms/bản tin |
| p | Tạm dừng gửi tự động; timer vẫn đếm. Lệnh này **không dừng động cơ** |
| r | Gửi một bản tin ngay |
| ? | In hướng dẫn |
| 1 / 0 | Bật / tắt LED LD2, giữ lại chức năng bài UART |
| e | Arm phần mềm, đặt PB0/ENA về mức RUN đã chọn; chưa phát xung |
| n<số> + Enter | Chọn số xung 1–80.000 khi IDLE, ví dụ n800; không chạy ngay |
| f / b | Chọn DIR thuận/ngược khi IDLE; không chạy ngay, không yêu cầu arm |
| g | Chạy một lần với số xung/chiều/tốc độ đã chọn; yêu cầu đã arm và IDLE |
| ! / d | Ép STEP LOW, đặt ENA STOP, disarm và xóa lệnh chờ; phải arm lại |
| v | Đảo mức ENA RUN/STOP khi DISARMED; đổi ngay mức chân STOP, không phát STEP; reset không lưu lựa chọn |
| + / - | Tăng/giảm 50 Hz cho lần chạy tiếp, chỉ khi IDLE; giới hạn 50–500 Hz |
| m | Đọc ARMED/ENA/DIR, pulses_done của lệnh trước/đang chạy, next_pulses và next_Hz cho lệnh tiếp |

`z` là đặt mốc tương đối, không phải homing và không tìm vị trí gốc robot. Mốc này mất khi reset hoặc mất nguồn.

## 5. Cách kiểm tra bằng động cơ của bạn

Thử một động cơ đã tách cơ cấu robot, sử dụng bộ phát xung điều khiển hiện có.

1. Khi trục đứng yên, gõ `z`. CNT phải giữ ổn định nếu trục thực sự không chuyển động.
2. Cho trục động cơ quay một vòng xác định: kỳ vọng độ lớn thay đổi khoảng **4000 count**, motor_deg khoảng **360°**, gear_deg khoảng **36°**.
3. Quay ngược lại về đúng vị trí ban đầu: CNT cần trở về gần 0. Đổi chiều quay phải đổi dấu thay đổi số đếm.
4. Một vòng trục ra hộp số 1:10 tương ứng khoảng **40000 count**, nếu đúng tỉ số truyền.
5. Lặp lại nhiều vòng và nhiều mức tốc độ để tìm sai số tích lũy. Có thể dùng logic analyzer đọc A1/A2 sau TXS để đối chiếu số cạnh.

800 xung lệnh/vòng trên driver và 4000 count encoder/vòng là hai đại lượng khác nhau.
Nếu dùng lệnh 800 xung để yêu cầu một vòng, cần cho động cơ bám xong và đứng yên; số xung lệnh không tự chứng minh trục đã quay đúng một vòng.
Sai lệch endpoint một vài count có thể do vị trí dừng cơ khí; không tự kết luận tất cả sai lệch là nhiễu module.

## 6. Chương trình hoạt động thế nào?

1. **TIM1** nhận PA8/PA9, chế độ TI1 and TI2: đếm cả bốn chuyển trạng thái mỗi chu kỳ A/B.
2. **SysTick 1 ms** đọc CNT 16 bit và tính chênh lệch có dấu, xử lý khi timer đi qua 65535 ↔ 0.
3. Chênh lệch được cộng vào số đếm 32 bit, rồi công bố snapshot atomic có số phiên bản. Điều kiện để giải cuộn đúng: dưới 32768 count giữa hai lần lấy mẫu. Chỉ SysTick ghi trạng thái tích lũy; `z` gửi yêu cầu để ISR thực hiện.
4. USART2 nhận phím bằng ngắt; `loop()` lấy lệnh từ ring buffer và gửi bản tin khoảng 100 ms/lần. Khi UART TX chờ, ngắt timer và RX vẫn hoạt động; encoder đếm bằng phần cứng.
5. Góc được tính bằng số nguyên 64 bit rồi in ba chữ số thập phân, không cần bật printf float hoặc printf long long của newlib-nano.

```text
góc động cơ [deg] = CNT × 360 / 4000
góc hộp số [deg] = CNT × 360 / 40000
rpm động cơ = delta_CNT × 60000 / (4000 × delta_t_ms)
```

Tốc độ là trung bình theo cửa sổ bản tin, chưa phải bộ ước lượng tốc độ cho NMPC. Count, A/B và thời gian trong bản tin thuộc cùng lượt lấy mẫu SysTick; CNT và GPIO vẫn được đọc lần lượt. Bản tin dùng mẫu hoàn chỉnh gần nhất, không đọc GPIO mới trong lúc in UART.
Khi debug Suspend, TIM1 được đóng băng; chuyển động xảy ra trong lúc đó không được ghi lại. Sau Resume, dừng trục và gõ z trước phép đo mới.

### Chỗ viết chương trình theo kiểu setup/loop

Mở `App/app.cpp`:

- `setup()`: khởi tạo STEP LOW, phần mềm DISARMED và ENA ở mức STOP cấu hình; khởi động encoder, bật UART RX, đặt cửa sổ tính tốc độ và in hướng dẫn; chạy một lần.
- `loop()`: xử lý thông báo UART/motor, xác nhận zero của encoder, các phím lệnh, LED và gửi dữ liệu đến hạn; được gọi liên tục.

`appTick1ms()` nằm trong `XX_encoder.cpp`, gọi `QuadratureEncoder::updateISR()`. Cờ atomic `dev.encoderReady` chỉ bật sau khi khởi tạo thành công, dùng release/acquire để công bố object. SysTick có thể chạy từ `HAL_Init()`, nhưng sẽ bỏ qua encoder khi cờ này chưa bật. `XX_serviceEncoder()` trong loop xử lý xác nhận zero, không chờ bận và không in UART từ ISR.

Trong `main.c`, sau phần khởi tạo CubeMX/BSP, chỉ cần:

```c
setup();
while (1)
{
    loop();
}
```

Các lời gọi này đã được đặt trong vùng `USER CODE`. Cách tổ chức tương tự setup/loop của Arduino, dùng C++ và STM32 HAL; không cần Arduino core.

### Sửa gì ở đâu?

| Muốn thay đổi | File |
|---|---|
| Thêm lệnh, đổi trình tự hoạt động | `App/app.cpp` |
| Count/vòng, tỉ số truyền, chu kỳ gửi, timeout UART | `App/systemConfig.h` |
| Xem trạng thái chung khi debug | Thêm `dev` vào Live Expressions; `dev.encoder` trỏ tới bộ đếm |
| Đổi định dạng bản tin hoặc cách giao tiếp | `App/XX_comm.cpp` |
| Chọn handle timer, nối hook ngắt | `App/XX_encoder.cpp` |
| Chọn handle/pin motor, callback TIM2 và báo trạng thái | `App/XX_motor.cpp` |
| Giá trị mặc định/giới hạn số xung, tần số, quy ước DIR, cực tính ENA | `App/systemConfig.h` (đổi mức khởi tạo PB0 trong `.ioc` tương ứng nếu đổi cực tính mặc định) |
| Số xung và tốc độ lần chạy kế tiếp trong RAM | `dev.motorPulseCount`, `dev.motorFrequency_hz`; dùng n... / +/- qua UART |
| Phát xung OPM và trạng thái arm/move/abort | `App/Libs/StepDirMotor.cpp` |
| Sửa thuật toán đếm/xử lý tràn timer | `App/Libs/QuadratureEncoder.cpp` |
| Clock, UART baud, pin và bộ lọc TIM1 | `Encoder_TXS.ioc` |

Snapshot/đặt mốc encoder dùng atomic và mailbox yêu cầu zero, không dùng PRIMASK.
Trạng thái tích lũy riêng của ISR không cần `volatile`; dữ liệu công bố và cờ
trao đổi dùng `std::atomic`. Motor/UART vẫn giữ các vùng tới hạn ngắn của chúng.
Chỉ `setup()`/`loop()` in UART; ISR không gọi printf, HAL_Delay hoặc truyền UART.

UART hiện nhận bằng `HAL_UART_Receive_IT`, có RxCplt/Error callback và khởi động lại khi lỗi trong `XX_comm.cpp`; truyền vẫn dùng HAL blocking có timeout. Cấu trúc hiện phục vụ đọc encoder và điều khiển một động cơ bằng STEP/DIR. Encoder được dùng để quan sát/đối chiếu, chưa có vòng điều khiển vị trí ngoài hoặc bộ giải NMPC trên MCU.

## 7. Cấu hình CubeMX đã lưu

| Tham số | Giá trị |
|---|---|
| MCU | STM32F446RE / NUCLEO-F446RE |
| Clock trong IOC hiện tại | HSE 8 MHz + PLL, SYSCLK 160 MHz, TIM1 160 MHz |
| USART2 | Asynchronous, PA2/PA3, 115200 8N1 |
| TIM1 encoder mode | TI1 and TI2 / TIM_ENCODERMODE_TI12 |
| TIM1 prescaler / counter period | 0 / 65535 |
| IC1/IC2 polarity | Rising (không đảo mức); encoder mode TI12 vẫn đếm ×4 |
| IC1/IC2 selection / prescaler | Direct TI / DIV1 |
| IC1/IC2 input filter | 12 |
| GPIO PA8/PA9 | AF1 TIM1, no pull-up / no pull-down |
| TIM2 | Internal clock, PWM CH1 mode 2, active-high, PSC=79, ARR=9999, CCR1=5000; thư viện bật OPM |
| PA0 / PA4 / PB0 | TIM2_CH1 STEP / DIR output / MOTOR_ENA output, PB0 khởi tạo HIGH và pull-up |
| Ưu tiên ngắt | Group 4: TIM2=1, USART2=2, SysTick=3 |
| Stack dự trữ trong linker / IOC | 0x1000 = 4096 byte |

Filter 12 tương ứng lấy mẫu fDTS/16 và yêu cầu 8 mẫu ổn định trên F446. Với clock TIM1 160 MHz theo IOC hiện tại, CKD DIV1, thời gian ổn định vào khoảng 0,8 µs.
Bộ lọc có thể bỏ xung nhiễu ngắn, nhưng không khắc phục sai mức điện áp, dây sai hoặc mất xung. Nếu đổi clock/tốc độ, phải xem lại bộ lọc.

Ngày 20/10/2026 đã Generate Code bằng CubeMX cho TIM2/GPIO/NVIC; các hook USER CODE được giữ lại. CubeMX đã bỏ `App` khỏi Source locations khi generate; cấu hình C++17/include/source/linker của Debug và Release đã được khôi phục từ bản sao trước generate.
Nếu Generate Code lại: giữ **Keep User Code**, giữ thư mục `App`, kiểm tra TIM1, USART2 và HAL_TIM_MODULE_ENABLED.
Các lời gọi ứng dụng và hook SysTick nằm trong USER CODE để CubeMX giữ lại. Sau khi generate, kiểm tra cả Debug/Release còn include `../App`, thư mục nguồn `App`, C++17 và G++ linker trỏ tới `STM32F446RETX_FLASH.ld`. Trong MCU G++ Compiler cần có cùng include và symbols `USE_NUCLEO_64`, `USE_HAL_DRIVER`, `STM32F446xx` như MCU GCC Compiler (Debug thêm `DEBUG`).

## 8. Những file cần đọc

- `App/app.cpp`: logic setup/loop và các phím lệnh.
- `App/app.h`: giao diện C, chỉ khai báo setup, loop và appTick1ms.
- `App/systemConfig.h`: hằng số có đơn vị và chân lấy từ label CubeMX.
- `App/device.h`: trạng thái dev và giao diện giữa các module.
- `App/XX_encoder.cpp`, `App/XX_comm.cpp`, `App/XX_motor.cpp`: ba module chức năng.
- `App/Libs/QuadratureEncoder.h/.cpp`: lớp encoder dùng lại được với handle/pin khác.
- `App/Libs/StepDirMotor.h/.cpp`: phát xung hữu hạn bằng PWM2/OPM, arm/move/abort.
- `Core/Src/main.c`: khởi tạo CubeMX/BSP và gọi ứng dụng.
- `Core/Src/stm32f4xx_hal_msp.c`: clock, chân PA8/PA9 encoder, PA0 STEP và NVIC.
- `Core/Src/stm32f4xx_it.c`: SysTick lấy mẫu encoder, IRQ TIM2/USART2 gọi HAL.
- `Encoder_TXS.ioc`: cấu hình CubeMX tương ứng.

## 9. Giới hạn của bài thử

TXS **đọc encoder** đang nhận một dây mỗi kênh so với mass; đây không phải mạch thu vi sai. Với VCCB=5,02 V, mức cao 4,5 V đã đo chưa đạt điều kiện VIH ≥ VCCB−0,4 V được TI bảo đảm.
OE của module encoder nối trực tiếp 3,3 V không bảo đảm đầu ra ở trạng thái Hi-Z trong thời gian nguồn đang lên/xuống. Bản hiện tại bỏ module TXS phát STEP/DIR; PB0 chuyển sang ENA+, không còn điều khiển OE.
Firmware không thể xác nhận phần cứng đạt chuẩn chỉ vì có count, OVF=0 hoặc UART hiển thị đúng. Cần đối chiếu xung/vòng và dạng sóng khi chạy trước khi dùng làm phản hồi điều khiển.

### Khi chưa thấy dữ liệu đúng

| Hiện tượng | Kiểm tra trước |
|---|---|
| Terminal không có chữ | Đúng COM, 115200 8N1, Flow control None; debugger đã Resume/F8; gõ ? hoặc s |
| Vẫn thấy LED ON/OFF của bài cũ | Có thể đang chạy Led. Chọn đúng Project Encoder_TXS và file Debug/Encoder_TXS.elf rồi nạp lại |
| Phím đơn chỉ nhận sau khi Enter | PuTTY Local line editing → Force off; riêng n<số> luôn cần Enter |
| Có bản tin nhưng CNT luôn 0 | Kiểm tra A1 TXS → D7/PA8, A2 TXS → D8/PA9; OE, hai nguồn và GND chung; đọc AB ở nhiều vị trí dừng |
| CNT chỉ dao động quanh 0 khi quay | Có thể một kênh không tới MCU hoặc hai dây bị trùng kênh; đối chiếu xung A/B bằng logic analyzer |
| CNT tự thay đổi khi trục thực sự đứng yên | Kiểm tra xung nhiễu, dây nối, mass và TXS; không suy ra bộ thu tốt chỉ từ số đo đồng hồ |
| Góc quay đúng độ lớn nhưng trái dấu mong muốn | Dấu phụ thuộc thứ tự A/B. Đây chưa phải lỗi PPR; ghi nhận chiều cơ khí trước khi quy ước dấu |
| Gõ f/b nhưng motor không quay | Đúng ở v3: f/b chỉ chọn chiều; dùng e rồi g sau khi đã đặt n... |
| MOTOR báo ARMED=0, g bị từ chối | Gõ e, đợi ARMED và thời gian ổn định, rồi gõ g; ENA_cmd không phải phản hồi trạng thái driver |
| ENA HIGH mà driver nhả mô men giữ | HIGH được dùng làm STOP; mặc định e đưa ENA LOW theo kết quả người dùng báo; kiểm tra ENA_run=LOW |
| Timer báo N/N nhưng motor không quay hoặc thiếu góc | Kiểm tra tín hiệu tại PUL khi có tải, enable/ALM của driver, mức DIR và cấu hình 800 xung/vòng; pulses_done không đo tại đầu vào driver |
| Gõ p mà động cơ vẫn quay | p chỉ tắt bản tin encoder; dùng ! hoặc d để ngắt STEP và đặt ENA STOP; mức này chỉ khóa driver khi cực tính và mức điện đã đúng |

## 10. Nguồn thư viện và tài liệu

- ST HAL/CMSIS và BSP: sao chép từ bài Led, cùng bộ STM32Cube FW_F4 V1.28.3; các file HAL TIM được bổ sung từ package này trên máy. Giữ nguyên giấy phép của ST trong Drivers.
- [ST HAL TIM](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/v1.8.5/Src/stm32f4xx_hal_tim.c)
- [STM32F446 reference manual RM0390](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [NUCLEO-64 UM1724](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
- [TXS0108E datasheet](https://www.ti.com/lit/ds/symlink/txs0108e.pdf)
