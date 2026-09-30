# Mạng ECU ô tô trên bàn qua CAN (ĐAKS)

Hướng đồ án chốt với GVHD ngày 30/9/2026. Đây là bản kiến trúc đầu tiên, **chưa có firmware**.
Hợp đồng giữa các ECU là file [`can/vehicle.dbc`](../can/vehicle.dbc) (bản nháp 0.1).

## 1. Mục tiêu

Mô phỏng một mạng ECU ô tô nhỏ trên bàn: 3 ECU vi điều khiển và 1 máy tính hiệu năng cao nối
chung một bus CAN, phần mềm chia tầng theo kiểu AUTOSAR (SWC → RTE → COM → CanIf → driver).
ECU trung tâm (VCU) chạy hệ điều hành OSEK (Trampoline). Dữ liệu cuối cùng hiển thị trên app
Android (thuộc ĐACN, làm riêng).

Không còn thuộc đồ án: xe AMR, jitter và điều khiển chuyển động, thí nghiệm E1–E4.

## 2. Kiến trúc hệ thống

```
 120 Ω                                                                          120 Ω
  ┌┴┐                                                                            ┌┴┐
══╪═══════════════╪═══════════════════╪═══════════════════╪═══════════════════════╪══  CANH/CANL
  │               │                   │                   │                       │    (dây xoắn đôi
 ECU1 VCU        ECU2 Pedal          ECU3 Body           Jetson Orin Nano             + GND chung)
 F407 + VP230    F103 + SN65HVD230   F103 + SN65HVD230   J17 + SN65HVD230
 Trampoline      bare-metal HAL      bare-metal HAL      Linux, SocketCAN
                                                             │ theo TÊN tín hiệu
                                                          Android HMI (ĐACN)
```

| Node | Phần cứng | Phần mềm | Gửi | Nhận |
|---|---|---|---|---|
| ECU1 VCU | Hiwonder ROS Robot Controller (STM32F407VET6), transceiver VP230 và trở 120 Ω R23 **có sẵn trên board** | Trampoline (OSEK): ISR2 nhận CAN, Alarm gửi theo chu kỳ, Resource bảo vệ dữ liệu chung | `VehSpeed`, `Battery` | `Pedal`, `BodyStatus` |
| ECU2 Pedal | STM32F103C8T6 + SN65HVD230. Biến trở = chân ga, nút = phanh, công tắc = số P/R/N/D | Bare-metal HAL + COM chung | `Pedal` | — |
| ECU3 Body | STM32F103C8T6 + SN65HVD230. Công tắc cửa, cần xi-nhan, công tắc đèn pha; LED xi-nhan, LED phanh | Bare-metal HAL + COM chung | `BodyStatus` | `Pedal` (lấy `Brake` để bật LED phanh) |
| Jetson (gateway) | Orin Nano, CAN qua header J17 + SN65HVD230 | Linux, SocketCAN (`mttcan`), Python + cantools | — (bản đầu chỉ nghe) | Tất cả |
| Android HMI | Điện thoại/tablet | App Kotlin (ĐACN, repo/thư mục riêng) | — | Tín hiệu theo tên, từ Jetson |

**Chức năng của VCU:**
- Mô hình xe đơn giản: ga làm tăng tốc; phanh và lực cản làm giảm tốc.
- Tính và gửi tốc độ, vòng quay mô-tơ, % pin (pin giảm theo tải).
- Buzzer (PA8) kêu khi cửa mở lúc xe đang chạy.
- Giám sát timeout của các ECU khác.

**Mở rộng tuỳ chọn:**
- ECU4 BMS.
- ECU ảo chạy Trampoline dạng POSIX trên Jetson. Trampoline **không** hỗ trợ Cortex-A78AE của Orin, chỉ chạy được dạng POSIX.

## 3. Nguyên tắc module (bắt buộc)

**a) DBC là hợp đồng chung duy nhất.** Thêm ECU nghĩa là thêm message vào DBC rồi sinh lại cấu hình. Không nơi nào khác được viết cứng ID, vị trí bit hay hệ số tỉ lệ.

**b) Mỗi ECU chia tầng như sau:**

```
┌──────────────────────────────┐
│ SWC (ứng dụng)               │  vehicle_model, door_warning, pedal_reader...
├──────────────────────────────┤
│ RTE tối giản                 │  Rte_Read_<Port>() / Rte_Write_<Port>()
├──────────────────────────────┤
│ COM          ◄── Com_Cfg.c/h │  đóng gói/bóc tách, chu kỳ, khi đổi, timeout
├──────────────────────────────┤     (Com_Cfg sinh tự động từ DBC)
│ CanIf                        │  PDU ↔ hộp thư CAN
├──────────────────────────────┤
│ Driver CAN (phụ thuộc chip)  │  bxCAN F407 / bxCAN F103
└──────────────────────────────┘
```

COM và CanIf viết bằng **C thuần, không gọi HAL**, dùng chung y hệt cho F407 và F103. Chỉ driver CAN phụ thuộc chip.

> F103 và F407 dùng **cùng một khối CAN (bxCAN)**, thanh ghi gần như giống hệt nhau. Driver có thể dùng chung, chỉ khác phần bật clock và cấu hình chân. Chuyện này để quyết khi viết driver.

**c) Dải ID theo miền:**

| Dải | Miền |
|---|---|
| 0x080–0x1FF | Truyền động và khung gầm |
| 0x300–0x3FF | Thân xe |
| 0x400–0x4FF | Pin, năng lượng |
| 0x500–0x5FF | HMI gửi ngược |
| 0x700–0x7FF | Chẩn đoán UDS |

ID nhỏ thắng khi tranh chấp bus, nên dữ liệu truyền động có ưu tiên cao nhất.

**d) Jetson và Android không phụ thuộc layout byte.** Jetson giải mã theo DBC và gửi đi theo tên tín hiệu, giống Vehicle HAL của Android Automotive. Nếu mất một ECU thì hiển thị "không có dữ liệu", không được báo lỗi hay treo.

## 4. CAN matrix (bản nháp 0.1)

Quy ước chung:
- CAN thường 500 kbit/s, ID 11 bit (F407 và F103 không có CAN FD).
- Mọi message dài **8 byte**, để thêm tín hiệu sau này mà không phải đổi DLC.
- Thứ tự byte kiểu Intel (little endian).
- Giá trị thô toàn bit 1 nghĩa là **SNA** (không có dữ liệu).

| ID | Gửi từ | Message | Chu kỳ | Timeout (đề xuất) | Tín hiệu |
|---|---|---|---|---|---|
| 0x080 | ECU2 | `Pedal` | 20 ms | 100 ms | `AccelPedal_pct`, `Brake`, `Gear` |
| 0x100 | ECU1 | `VehSpeed` | 20 ms | 100 ms | `VehicleSpeed`, `MotorRpm` |
| 0x300 | ECU3 | `BodyStatus` | 100 ms, và gửi ngay khi đổi (cách nhau tối thiểu 20 ms) | 500 ms | `DoorFL/FR/RL/RR`, `TurnLeft`, `TurnRight`, `Headlight` |
| 0x400 | ECU1 | `Battery` | 500 ms | 2500 ms | `Soc_pct`, `BattVoltage`, `LowBattWarn` |

Timeout lấy bằng 5 chu kỳ. Đây là con số khởi điểm, sẽ đo thời gian phát hiện mất ECU để chỉnh lại.

| Tín hiệu | Bit | Độ dài | Tỉ lệ, đơn vị | Dải | Giả định |
|---|---|---|---|---|---|
| `AccelPedal_pct` | 0 | 8 | 0,5 % | 0–100 | 255 = SNA (ví dụ đứt dây biến trở) |
| `Brake` | 8 | 1 | — | 0/1 | 1 = đang đạp |
| `Gear` | 16 | 3 | enum | 0 P, 1 R, 2 N, 3 D, 7 SNA | |
| `VehicleSpeed` | 0 | 16 | 0,01 km/h | 0–250 | Độ lớn, không dấu; chiều chạy suy từ `Gear` |
| `MotorRpm` | 16 | 16 có dấu | 1 rpm | ±12000 | Âm khi lùi; 0x8000 = SNA |
| `DoorFL…DoorRR` | 0–3 | 1 mỗi cửa | — | 0/1 | 1 = mở. Bản đầu chỉ có 1 công tắc, nối vào `DoorFL` |
| `TurnLeft`, `TurnRight` | 8, 9 | 1 | — | 0/1 | Trạng thái **cần gạt**, không phải đèn đang sáng. ECU3 tự nháy LED, HMI tự nháy biểu tượng |
| `Headlight` | 16 | 2 | enum | 0 tắt, 1 cos, 2 pha, 3 SNA | Bản đầu chỉ có công tắc bật/tắt |
| `Soc_pct` | 0 | 8 | 0,5 % | 0–100 | |
| `BattVoltage` | 8 | 16 | 0,1 V | 0–500 | Bộ pin xe điện mô phỏng khoảng 350–400 V |
| `LowBattWarn` | 24 | 1 | — | 0/1 | Ngưỡng đặt trong VCU, dự kiến 20 % |

Byte 6–7 của `Pedal` để trống, dự phòng cho E2E (counter + CRC) nếu chọn phần mở rộng đó.
`AccelPedal_pct`, `Brake`, `Gear` gom thành signal group `PedalGroup`, để bên nhận luôn đọc được bộ 3 giá trị nhất quán.

**Tải bus lý thuyết** (đã kiểm bằng cantools): mỗi khung 8 byte, xấu nhất 135 bit (tính cả bit stuffing), 112 khung/s → **15 120 bit/s = 3,0 %** của 500 kbit/s. Dư rất nhiều chỗ cho ECU4 hoặc UDS.

## 5. Phần cứng

| Có sẵn | Cần mua |
|---|---|
| Board Hiwonder (F407 + VP230 + 120 Ω) | 1× STM32F103C8T6 (đã có 1, cần 2) |
| 1× STM32F103C8T6 | 3× module SN65HVD230 (2 cho F103, 1 cho Jetson) |
| Jetson Orin Nano | Dây xoắn đôi |
| ST-Link | Hàng rào chân cho J17 nếu chưa hàn |

## 6. Cách đấu bus

1. **Đi dây thẳng một đường** (không đấu hình sao), nhánh rẽ tới từng node càng ngắn càng tốt (dưới khoảng 30 cm là ổn ở 500 kbit/s trên bàn).
2. **Đúng 2 trở 120 Ω, đặt ở 2 đầu bus:**
   - Đầu thứ nhất là ECU1, vì trở R23 hàn cố định trên board Hiwonder.
   - Đầu còn lại là node xa nhất (đề xuất Jetson).
   - Các module SN65HVD230 thường có sẵn trở 120 Ω trên module: **gỡ ra** ở mọi node không nằm ở đầu bus.
3. **Kiểm tra trước khi cấp điện:** tắt hết nguồn, đo điện trở giữa CANH và CANL. Phải được **khoảng 60 Ω** (hai trở 120 Ω song song). Ra 40 Ω nghĩa là thừa một trở; ra 120 Ω nghĩa là thiếu một trở.
4. **Dây GND chung** giữa mọi node, đi cùng cặp CANH/CANL.
5. **Cấp nguồn các node từ cùng một nguồn hoặc một hub.** Đây là bài học từ các lần hỏng board trước (ground loop, sạc laptop 2 chân): chỉ một đường nguồn, rút sạc laptop khi thao tác trên board hở.

| Node | Chân CAN | Ghi chú |
|---|---|---|
| ECU1 | CAN1: PD0 = RX, PD1 = TX, đã nối sẵn tới VP230. CANH/CANL ra ở header H1 | Xác nhận lại vị trí CANH/CANL trên board thật |
| ECU2, ECU3 | CAN1 **remap sang PB8 = RX, PB9 = TX** | Không dùng PA11/PA12 vì đó là chân USB D−/D+ của Blue Pill (PA12 có trở kéo lên 1,5 kΩ). Trên F103, USB và CAN còn dùng chung một vùng SRAM nên không chạy đồng thời được |
| Jetson | Header J17 (CAN_TX, CAN_RX mức 3,3 V) → SN65HVD230 | Cần xác nhận pinout J17 và việc bật pinmux cho `mttcan` trên JetPack 6. Lệnh dự kiến: `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up`, xem bằng `candump can0` |

SN65HVD230 dùng nguồn 3,3 V, khớp mức logic của STM32 và Jetson.

**Bit timing:** mọi node phải cùng 500 kbit/s **và cùng điểm lấy mẫu** (khoảng 87,5 %). Clock CAN khác nhau giữa các chip (F407 APB1 = 42 MHz, F103 APB1 = 36 MHz), nên prescaler và số time quanta sẽ tính riêng cho từng chip khi viết driver.

## 7. Đề xuất cấu trúc thư mục — CHỜ DUYỆT

Chưa tạo thư mục nào ngoài `can/`.

```
can/
  vehicle.dbc              hợp đồng duy nhất giữa các ECU
tools/
  comgen/                  Python: đọc DBC → sinh Com_Cfg.c/.h + Rte_<Ecu>.h cho từng ECU
    templates/
    tests/
bsw/                       C thuần, KHÔNG gọi HAL, dùng chung mọi ECU
  include/                 Std_Types.h, ComStack_Types.h
  com/                     Com.c/.h: đóng gói/bóc tách, gửi theo chu kỳ/khi đổi, deadline, signal group
  canif/                   CanIf.c/.h: ánh xạ PDU ↔ hộp thư CAN, gọi driver qua Can_Write()
drivers/
  can_bxcan/               driver bxCAN; dùng chung F103/F407, hoặc tách 2 thư mục nếu khác nhau nhiều
ecu1_vcu/                  app Trampoline
  vcu.oil
  swc/                     vehicle_model.c, door_warning.c, timeout_monitor.c
  board/                   phần riêng của board Hiwonder (clock, GPIO, buzzer)
  gen/                     Com_Cfg.c/.h, Rte_Vcu.h do comgen sinh ra
ecu2_pedal/                project STM32CubeIDE cho F103
  swc/  gen/
ecu3_body/                 project STM32CubeIDE cho F103
  swc/  gen/
jetson_gateway/            Python: SocketCAN + cantools → gửi theo tên tín hiệu
tests/
  host/                    unit test COM/CanIf chạy trên PC (gcc), không cần board
  bus/                     kịch bản python-can: giả lập lỗi, gửi message sai, đo độ trễ
```

**Cần bạn quyết:**
1. **Vị trí:** để các thư mục ở gốc repo `amr_ws` như trên, hay gom vào một thư mục `can_network/`, hay tách hẳn repo mới? Repo hiện tại chủ yếu là code xe AMR, không còn thuộc đồ án. Tách repo thì báo cáo và mã nguồn đồ án gọn hơn.
2. **File sinh tự động (`gen/`):** commit vào git (build firmware không cần Python; comgen ghi dòng "KHÔNG SỬA TAY" ở đầu file), hay sinh lại mỗi lần build.
3. **RTE:** đề xuất sinh `Rte_<Ecu>.h` từ một file ánh xạ nhỏ cho mỗi ECU (port của SWC ↔ tín hiệu trong DBC).
4. **Driver CAN:** một driver bxCAN dùng chung hay mỗi chip một driver. Nên quyết sau khi đọc reference manual của cả hai chip.

Có thể tham khảo lại cấu hình CubeMX của firmware F103 cũ (HSE 8 MHz → 72 MHz, SYS Debug = Serial Wire). Thư mục đó đã xoá khỏi repo nhưng còn trong lịch sử git: `git show 00a0954:amr_stm32f103/amr_stm32f103.ioc`.

## 8. Việc chưa xác định, rủi ro

- **Trampoline chưa có driver CAN cho stm32f407.** Phải tự viết ISR2 cho CAN1 RX, cộng với phần ghép HAL/CMSIS vào Trampoline (việc này đã ghi trong `docs/trampoline-setup.md`, mục "Việc tiếp theo").
- **Jetson J17:** chưa xác nhận pinout, pinmux và mức điện áp trên board thật.
- **Board Hiwonder:** chưa kiểm tra ngoài đời. Vẫn dùng checklist `docs/bringup-f407.md` cho phần nguồn, nạp code, LED, buzzer; phần motor và servo trong checklist đó không cần nữa.
- **Chiều truyền Jetson → Android** (WebSocket, UDP hay khác): quyết cùng ĐACN.

## 9. Đánh giá (dự kiến)

- Test chức năng từng tín hiệu, từ đầu vào vật lý đến màn hình HMI.
- Tải bus: tính từ DBC (mục 4) so với đo thực tế bằng `candump`.
- Độ trễ đầu–cuối: từ lúc đạp ga (ECU2) đến lúc `VehSpeed` đổi (ECU1), và đến lúc Jetson nhận được.
- Thời gian phát hiện mất ECU (so với timeout đặt trong DBC).
- Giả lập lỗi: rút một node, gửi message sai độ dài hoặc sai giá trị.
- Phần mở rộng (chọn sau): UDS chẩn đoán (0x7E0/0x7E8) hoặc E2E (counter + CRC) cho `Pedal`.
