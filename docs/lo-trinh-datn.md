# Lộ trình ĐATN: OSEK/Trampoline trên AMR

> ⚠️ **KHÔNG CÒN THUỘC ĐỒ ÁN (từ 30/9/2026).** Đề tài đã đổi sang mạng ECU ô tô trên bàn qua CAN,
> xem `docs/can-network-architecture.md`. Giữ file này để tham khảo: phần lý thuyết OSEK/Trampoline
> (GĐ 1B, 1C, 2) và toolchain (GĐ 0) vẫn dùng được; phần gắn với xe (jitter, baseline trên xe, E1–E4) thì bỏ.

**Đề tài:** Phân tích ảnh hưởng của đặc tính thời gian thực trên ECU dùng OSEK/AUTOSAR OS
(Trampoline) đến chất lượng điều khiển chuyển động của AMR lái Ackermann.

**Câu hỏi nghiên cứu:** khi chuyển firmware từ superloop sang OSEK, hoặc khi thay đổi chu kỳ hay
tải CPU, thì jitter của vòng điều khiển thay đổi bao nhiêu, và sai số vận tốc, heading, quỹ đạo
của xe thay đổi theo ra sao?

> Lộ trình này bám theo đề cương `reference/ĐA/DAKS — OSEK Trampoline trên AMR.pdf` (bản
> 2026-09-28), đã chỉnh lại theo phần cứng mới (board Hiwonder F407, xem cuối file).

---

## Cách làm việc giữa bạn và Claude

Mỗi giai đoạn đi đúng thứ tự:

```
ĐỌC ──► HIỂU ──► THIẾT KẾ ──► CODE ──► TEST ──► GHI LẠI
 bạn    tự trả    Claude giảng,   bạn viết,  bạn chạy,   nhật ký +
        lời câu   bạn viết bản    Claude     Claude      nháp chương
        hỏi       thiết kế        review     giải thích  báo cáo
```

Mỗi đầu việc được gắn một trong ba mức tham gia:

| Ký hiệu | Nghĩa |
|---|---|
| 🧑 **Bạn làm** | Bạn tự làm. Claude chỉ gợi ý khi bạn bị kẹt, không viết hộ |
| 🤝 **Làm cùng** | Claude giải thích hoặc viết khung, bạn viết phần chính rồi Claude review |
| 🤖 **Claude làm, bạn giải thích lại** | Phần kỹ thuật sâu, ít giá trị học (vá build, xung đột thư viện). Claude làm, sau đó **bạn phải giải thích lại được bằng lời của mình** thì mới coi là xong |

**Quy tắc:**
- Không chuyển giai đoạn khi chưa trả lời được các câu hỏi tự kiểm tra của giai đoạn đó.
- Mỗi buổi làm, ghi 3–5 dòng vào `docs/nhat-ky/` (ngày, làm gì, gặp gì, học được gì). Đây là nguyên liệu cho báo cáo và để trả lời hội đồng.
- Mọi bước nạp code hoặc làm motor quay, bạn xác nhận trước (như quy tắc cũ).

---

## Tổng quan các giai đoạn

| GĐ | Nội dung | Cần board? | Ước tính | Sản phẩm cho báo cáo |
|---|---|---|---|---|
| 0 | Rà lại phần Claude đã làm trước | Không | 3–4 h | Bạn tự build được; đoạn "Môi trường phát triển" |
| 1 | Lý thuyết nền | Không | 15–20 h | Nháp **Chương 2: Cơ sở lý thuyết** |
| 2 | Hiểu Trampoline qua code và bài tập | Không | 10–12 h | Sơ đồ "từ ngắt SysTick tới task chạy" |
| 3 | Thiết kế hệ thống và thí nghiệm | Không | 12–15 h | Nháp **Chương 3: Thiết kế** |
| 4 | Bring-up board + đo baseline superloop | **Có** | 10–15 h | Bảng số liệu superloop (dùng chung với ĐACN) |
| 5 | Tích hợp target Hiwonder cho Trampoline | Có (ở cuối) | 10–20 h | LED nháy bằng Alarm, UART bằng ISR2 |
| 6 | Chuyển firmware sang các task | Có | 30–40 h | Xe chạy như bản superloop; nháp **Chương 4** |
| 7 | Thí nghiệm E1, E2 và phân tích | Có | 35–45 h | Biểu đồ, bảng; nháp **Chương 5** |
| 8 | Viết báo cáo, slide, tập bảo vệ | Không | 50–70 h | Bản nộp |

GĐ 0–3 làm được **ngay trong 2 tuần chờ board**. Đây là lợi thế: khi board về, bạn đã có thiết kế trong tay và chỉ còn việc triển khai.

**Mốc go/no-go** (giữ theo đề cương):
- **Cuối T7:** GĐ 5 phải xong (LED nháy bằng Alarm trên board thật). Nếu chưa thì chuyển sang phương án dự phòng.
- **Cuối T15:** dừng mọi thí nghiệm mới, chỉ còn viết báo cáo.

---

## GĐ 0: Rà lại phần Claude đã làm trước (3–4 h)

Trong lúc bạn chưa kịp tham gia, Claude đã làm trước **bước 1 của đề cương** (cài goil, build blink) và **một phần bước 3** (bật hard-float). Giai đoạn này là để bạn nắm lại, không coi như chưa có gì.

| # | Việc | Mức |
|---|---|---|
| 0.1 | Đọc `docs/trampoline-setup.md` | 🧑 |
| 0.2 | Tự chạy `scripts/trampoline_build.sh amr_trampoline/fpu_check` trong Git Bash, xem các file goil sinh ra trong `amr_trampoline/fpu_check/fpu_check/` | 🧑 |
| 0.3 | Đọc 2 file trong `amr_trampoline/patches/`. Với mỗi chỗ sửa, trả lời: *sửa gì, vì sao, không sửa thì chuyện gì xảy ra* | 🤖 → bạn giải thích lại |
| 0.4 | Viết nháp mục "Môi trường phát triển" cho báo cáo (toolchain, phiên bản, các bản vá) | 🧑 |

**Câu hỏi tự kiểm tra:**
- goil là gì? Đầu vào và đầu ra của nó là gì?
- Vì sao cần 2 bộ GCC (WinLibs và arm-none-eabi)? Mỗi bộ biên dịch cái gì?
- Soft-float khác hard-float thế nào? Vì sao để lệch nhau thì thí nghiệm E1 mất ý nghĩa?

---

## GĐ 1: Lý thuyết nền (15–20 h)

Chia 4 chủ đề, học theo thứ tự. Mỗi chủ đề có tài liệu, bài tập nhỏ và câu hỏi tự kiểm tra.

### 1A. Hệ thời gian thực và lập lịch (5–6 h)

**Khái niệm cần nắm:** task tuần hoàn; chu kỳ T, thời gian thực thi C, deadline D; WCET; jitter; response time; CPU utilization U; lập lịch có ưu tiên cố định; preemptive và non-preemptive; Rate Monotonic (chu kỳ ngắn thì ưu tiên cao); giới hạn Liu & Layland; **phân tích thời gian đáp ứng (RTA)**:

```
R_i = C_i + Σ_{j ưu tiên cao hơn i} ⌈R_i / T_j⌉ · C_j    (lặp tới khi hội tụ)
```

**Tài liệu:**
- G. Buttazzo, *Hard Real-Time Computing Systems*, chương 1–4 (sách giáo khoa chuẩn, trích dẫn được trong báo cáo)
- C. L. Liu & J. Layland (1973), bài báo gốc về Rate Monotonic, chỉ cần đọc phần kết quả

**Bài tập** 🧑: với 3 task giả định (T = 10/20/50 ms, C = 1/3/10 ms), tính U, kiểm tra giới hạn Liu & Layland, rồi tính RTA bằng tay. Claude sẽ chấm.

### 1B. Chuẩn OSEK/VDX OS (5–6 h)

**Khái niệm cần nắm:**
- Task basic và extended; 4 trạng thái (suspended/ready/running/waiting)
- Conformance class BCC1…ECC2
- Scheduling `FULL` và `NON`
- Counter và Alarm
- ISR category 1 và 2
- Resource và giao thức **Priority Ceiling (OSEK PCP)**, dùng để chống đảo ưu tiên
- Hook
- Cấu hình tĩnh bằng OIL

**Tài liệu** (có sẵn trong `Documents/trampoline/documentation/`):
- `qrdc/osek_qrdc.pdf` và `qrdc/oil_qrdc.pdf`: tóm tắt 1–2 trang, **đọc đầu tiên**
- `manual/main.pdf`: sổ tay Trampoline, các chương task, scheduling, alarm, resource, ISR
- Chuẩn OSEK/VDX OS 2.2.3 (tài liệu gốc, trích dẫn trong báo cáo)

**Bài tập** 🧑: vẽ sơ đồ trạng thái task và đánh dấu API nào gây ra mỗi chuyển trạng thái (`ActivateTask`, `TerminateTask`, `WaitEvent`, `SetEvent`).

### 1C. Cortex-M4 bên dưới hệ điều hành (3–4 h)

**Khái niệm cần nắm:**
- SysTick
- NVIC và mức ưu tiên ngắt
- SVC và PendSV
- Chuyển ngữ cảnh: thanh ghi nào được lưu, lưu vào đâu
- Mỗi task có stack riêng
- Ngữ cảnh FPU (s0–s31) và lazy stacking
- Bộ đếm chu kỳ DWT CYCCNT

**Tài liệu:**
- J. Yiu, *The Definitive Guide to ARM Cortex-M3 and Cortex-M4 Processors*, các chương về exception và hỗ trợ hệ điều hành
- ST PM0214 (programming manual Cortex-M4)

**Liên hệ với code:** `machines/cortex-m/armv7em/tpl_ctx_switch.S`. Đọc cùng Claude ở GĐ 2.

### 1D. Jitter ảnh hưởng tới điều khiển như thế nào (2–3 h)

Đây là cầu nối giữa "RTOS" và "xe chạy". Nó là lý do đề tài có ý nghĩa và sẽ bị hội đồng hỏi.

**Khái niệm cần nắm:**
- Vòng điều khiển số giả định lấy mẫu đều đặn
- Jitter lấy mẫu và trễ vòng lặp làm giảm độ dự trữ pha (phase margin)
- Vì sao PID của mình dùng `dt` cố định 10 ms, nên khi jitter lớn thì khâu I bị tính sai

**Tài liệu:** A. Cervin et al., *How does control timing affect performance?*, IEEE Control Systems Magazine, 2003.

**Sản phẩm GĐ 1** 🧑: bảng thuật ngữ (viết bằng lời của bạn) và nháp Chương 2. Claude review nội dung, không viết hộ.

**Câu hỏi tự kiểm tra (kiểu hội đồng hỏi):**
- Jitter khác response time chỗ nào? Cho ví dụ bằng số.
- Tại sao superloop lại có jitter? Chỉ ra nguyên nhân cụ thể trong `main.c` của mình.
- Preemption giảm jitter cho task ưu tiên cao, vậy cái giá phải trả là gì? (Gợi ý: task ưu tiên thấp, dữ liệu dùng chung, overhead chuyển ngữ cảnh.)
- Nếu hai task cùng ghi biến `target_speed` thì chuyện gì xảy ra? OSEK giải quyết bằng cơ chế nào?

---

## GĐ 2: Hiểu Trampoline qua code và bài tập (10–12 h)

| # | Việc | Mức |
|---|---|---|
| 2.1 | Đọc `fpu_check.oil` và `fpu_check.c` cùng Claude. Ánh xạ từng khối trong OIL sang code C mà goil sinh ra (`tpl_app_config.c`) | 🤝 |
| 2.2 | Lần theo đường đi: ngắt SysTick → counter → alarm hết hạn → `ActivateTask` → scheduler chọn task → chuyển ngữ cảnh. Claude chỉ file và hàm, bạn đọc và vẽ sơ đồ | 🤝 |
| 2.3 | **Bài tập 1:** tự viết app mới có 2 task tuần hoàn (10 ms, 50 ms), mỗi task tăng một biến đếm. Build, rồi xem file `.map` để biết stack mỗi task nằm ở đâu | 🧑 |
| 2.4 | **Bài tập 2:** thêm một Resource dùng chung giữa 2 task. Giải thích goil đã tính "ceiling priority" bằng bao nhiêu | 🧑 |
| 2.5 | **Bài tập 3:** đổi `SCHEDULE = FULL` thành `NON` cho một task. Dự đoán hành vi thay đổi ra sao, rồi kiểm chứng lại (bằng suy luận trên code, vì chưa có board) | 🧑 |

**Sản phẩm:** sơ đồ "từ ngắt SysTick tới task chạy" do bạn vẽ, sau này đưa vào Chương 2 hoặc Chương 4.

**Câu hỏi tự kiểm tra:**
- goil sinh ra những file nào? File nào chứa bảng task? File nào chứa bảng vector ngắt?
- Khi task `fast` cắt ngang `slow`, những thanh ghi nào được lưu, và lưu vào biến nào?
- Vì sao ISR trong dự án này không được dùng float?

---

## GĐ 3: Thiết kế hệ thống và thí nghiệm (12–15 h)

Đây là giai đoạn **Claude giảng giải nhiều nhất**, nhưng **bạn là người viết** tài liệu thiết kế `docs/thiet-ke-osek.md`. Tài liệu này sẽ thành Chương 3.

| # | Việc | Mức |
|---|---|---|
| 3.1 | Cùng đọc `amr_stm32f407/Core/Src/main.c`, liệt kê **mọi việc** superloop đang làm: tên việc, chu kỳ, ước lượng thời gian, dữ liệu đọc/ghi | 🤝 |
| 3.2 | Chia thành các task (đề cương: Control, IMU, Comm, Diag). Với mỗi task quyết định: chu kỳ, ưu tiên, kích hoạt bằng Alarm hay bằng ISR2, dữ liệu dùng chung, có `USEFLOAT` không | 🤝, **bạn ra quyết định và giải thích lý do** |
| 3.3 | Xử lý các điểm an toàn: watchdog `$VEL` đặt ở đâu; lệnh dừng (`target = 0`) phải nằm ngoài vòng PID như đã học; Resource cho dữ liệu dùng chung | 🤝 |
| 3.4 | **Phân tích trên giấy:** tính U và RTA cho tập task vừa thiết kế, từ đó đưa ra **giả thuyết** cho E1 (dự đoán jitter của Control sẽ ra sao) | 🧑, Claude chấm |
| 3.5 | Thiết kế phép đo: đặt điểm DWT ở đâu, ring buffer, vì sao không gửi dữ liệu trong lúc đo (probe effect), kiểm tra chéo bằng GPIO + logic analyzer | 🤝 |
| 3.6 | Thiết kế thí nghiệm E1 và E2: biến thay đổi, biến giữ cố định, đại lượng đo, bài chạy (đi thẳng, vòng tròn bẻ hết lái), **số lần lặp**, cách tính thống kê | 🤝 |
| 3.7 | Mang bản thiết kế đi trao đổi với giảng viên hướng dẫn và chốt phạm vi | 🧑 |

**Câu hỏi tự kiểm tra:**
- Vì sao Control có ưu tiên cao nhất? Nếu đảo thứ tự (thí nghiệm E4) thì dự đoán điều gì xảy ra?
- Comm nên kích hoạt theo chu kỳ hay theo sự kiện nhận UART? Mỗi cách có ưu và nhược điểm gì?
- Làm sao chứng minh bản thân việc đo (DWT) không làm sai kết quả?

---

## GĐ 4: Bring-up board + đo baseline superloop (10–15 h, cần board)

Phần này dùng chung với ĐACN.

| # | Việc | Mức |
|---|---|---|
| 4.1 | Chạy checklist `docs/bringup-f407.md` phần A → D. Bạn thao tác trên board, gửi kết quả, Claude điền hằng số vào code | 🧑 thao tác, 🤝 điền code |
| 4.2 | Kiểm tra PID trên bàn (nhấc bánh), rồi chạy thử trên sàn (đi thẳng, vòng tròn) | 🧑, Claude hỗ trợ |
| 4.3 | Đo baseline bằng DWT (port hạ tầng `baseline-dwt` sang F407), phân tích bằng Python | 🤝 |
| 4.4 | Chạy `fpu_check` trên board thật: `slow_err_count` phải luôn bằng 0 | 🧑 |

**Tiêu chí xong:** có bảng period, jitter, execution time của PID trong superloop.

---

## GĐ 5: Tích hợp target Hiwonder cho Trampoline (10–20 h)

Tương ứng bước 2 và 3 của đề cương. Rủi ro đã **giảm nhiều** so với đề cương, vì F407 là target chính thức (không phải tự port F411 như kế hoạch cũ).

| # | Việc | Mức |
|---|---|---|
| 5.1 | Tạo board Hiwonder trong Trampoline (LED PE10, buzzer PA8) thay cho BSP Discovery | 🤝 |
| 5.2 | Bỏ StdPeriph, ghép HAL. Xử lý 3 điểm: header CMSIS trùng nhau, `HAL_GetTick()` khi OS đã chiếm SysTick, chỉ cấu hình clock ở một chỗ | 🤖 → bạn giải thích lại |
| 5.3 | Viết OIL + task nháy LED bằng Alarm, build, nạp, đo chu kỳ LED | 🧑 |
| 5.4 | ISR2 cho USART3: máy tính gửi 1 ký tự → task được kích hoạt và phản hồi | 🤝 |

**Tiêu chí xong (mốc go/no-go cuối T7):** LED nháy đúng chu kỳ bằng Alarm trên board thật, và UART echo qua ISR2 chạy được.

---

## GĐ 6: Chuyển firmware sang các task (30–40 h)

Chuyển **từng task một**, mỗi lần chuyển xong phải test trên bàn (nhấc bánh) trước khi làm task tiếp theo. Thứ tự: Control → IMU → Comm → Diag.

| # | Việc | Mức |
|---|---|---|
| 6.1 | Task Control: encoder → PID → PWM, `USEFLOAT = TRUE` | 🧑 viết, Claude review |
| 6.2 | Task IMU (MPU-6050, I2C2) | 🧑 viết, Claude review |
| 6.3 | Task Comm: nhận `$VEL` qua ISR2, gửi `$ODO` (tái sử dụng driver UART-DMA của ĐACN) | 🤝 |
| 6.4 | Task Diag + watchdog an toàn | 🧑 viết, Claude review |
| 6.5 | Chèn các điểm đo DWT giống hệt bản superloop | 🤝 |
| 6.6 | Chạy lại đúng các bài đi thẳng và vòng tròn như lúc hiệu chuẩn | 🧑 |

**Tiêu chí xong:** xe chạy được như bản superloop, kết quả bài đi thẳng và vòng tròn tương đương.

---

## GĐ 7: Thí nghiệm E1, E2 và phân tích (35–45 h)

| Thí nghiệm | Biến thay đổi | Giữ cố định | Mức ưu tiên |
|---|---|---|---|
| **E1** Kiến trúc | Superloop và OSEK | Chu kỳ 10 ms, tải bình thường | Core |
| **E2** Tải CPU | Task giả tải chiếm 0 / 30 / 60 % CPU | OSEK, chu kỳ 10 ms | Core |
| E3 Chu kỳ | Control chạy 5 / 10 / 20 ms | OSEK | Optional |
| E4 Ưu tiên | Ưu tiên đúng và bị đảo | OSEK, tải 60 % | Optional |

| # | Việc | Mức |
|---|---|---|
| 7.1 | Chạy thí nghiệm theo đúng thiết kế ở GĐ 3.6, ghi nhật ký từng lần chạy | 🧑 |
| 7.2 | Script Python phân tích và vẽ biểu đồ | 🤝 (Claude viết khung, bạn hiểu và chỉnh) |
| 7.3 | So kết quả đo với giả thuyết RTA ở GĐ 3.4. Chỗ nào khác, giải thích vì sao | 🧑, Claude thảo luận |

**Kết luận cần rút ra:** jitter của Control giảm bao nhiêu khi chuyển sang OSEK (E1), và từ mức tải nào thì sai số chuyển động bắt đầu tăng rõ (E2).

---

## GĐ 8: Báo cáo, slide, tập bảo vệ (50–70 h)

| Chương | Lấy từ |
|---|---|
| 1. Tổng quan, đặt vấn đề | Đề cương, câu hỏi nghiên cứu |
| 2. Cơ sở lý thuyết | GĐ 1, GĐ 2 |
| 3. Thiết kế hệ thống và thí nghiệm | GĐ 3 |
| 4. Triển khai | GĐ 0, 5, 6 và nhật ký |
| 5. Kết quả và thảo luận | GĐ 4, 7 |
| 6. Kết luận, hướng phát triển | Phần optional chưa làm (E3/E4, AUTOSAR SWC, CAN) |

**Tập bảo vệ** 🤝: Claude đóng vai hội đồng, hỏi dồn các câu tự kiểm tra ở mọi giai đoạn.

---

## Những điểm đề cương (bản 2026-09-28) cần cập nhật với GVHD

- **Phần cứng:** F411 + BNO055 → **Hiwonder ROS Robot Controller (F407) + MPU-6050 on-board**. Việc "port target F411" (rủi ro lớn nhất) thu gọn lại thành "thêm board F407". Dòng "Ngoài phạm vi: chuyển sang F407" trong đề cương không còn đúng.
- **Heading:** MPU-6050 không có từ kế, nên yaw bị trôi. Nếu E1/E2 đo sai số heading thì cần tính tới điều này, hoặc gắn thêm BNO055 qua cổng I2C số 5.
- **CAN (optional):** board có sẵn transceiver VP230, không cần MCP2515.
- Phương án dự phòng "mua board Discovery" gần như không còn cần, vì board mới dùng cùng chip F407.

## Trạng thái

- [x] Bước 1 đề cương (goil, blink): Claude đã làm trước, **chờ bạn rà lại ở GĐ 0**
- [x] Một phần bước 3 (hard-float, `USEFLOAT`): Claude đã làm trước, **chờ bạn rà lại ở GĐ 0**
- [ ] GĐ 0 · [ ] GĐ 1 · [ ] GĐ 2 · [ ] GĐ 3 · [ ] GĐ 4 · [ ] GĐ 5 · [ ] GĐ 6 · [ ] GĐ 7 · [ ] GĐ 8
