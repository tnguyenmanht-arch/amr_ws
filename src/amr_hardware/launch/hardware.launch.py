#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # CẦN ĐIỀU CHỈNH serial_port theo thiết bị thực tế
    # Kiểm tra bằng: ls /dev/ttyUSB* /dev/ttyACM*
    serial_port = LaunchConfiguration('serial_port')
    baud_rate   = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Port serial kết nối STM32'),
        DeclareLaunchArgument('baud_rate',   default_value='115200',
                              description='Baud rate — phải khớp với STM32 firmware'),
        Node(
            package='amr_hardware',
            executable='serial_driver_node',
            name='serial_driver_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate':   baud_rate,
                # ⚠️ HIỆU CHUẨN LẠI TOÀN BỘ 2026-09-05 — giá trị cũ
                # (wheel_radius=0.10, encoder_ppr=87.61) là HAI LỖI SAI BÙ TRỪ
                # NHAU, không phải calibration đúng:
                #   - wheel_radius=0.10 là ĐƯỜNG KÍNH bị ghi nhầm thành bán
                #     kính. Bánh xe thật: đường kính 100mm -> BÁN KÍNH 0.05m.
                #     (Dấu hiệu nhận ra: cả xe chỉ cao 84mm, bánh bán kính
                #     100mm sẽ to hơn cả xe -> vô lý.)
                #   - encoder_ppr=87.61 khi đó phải gấp đôi giá trị thật để
                #     bù cho bán kính gấp đôi, nên /odom vẫn ra đúng quãng
                #     đường. Chỉ tích (2*pi*r / ticks_per_rev) là được hiệu
                #     chuẩn thật, còn từng hằng số riêng đều sai.
                #
                # Tham số hoá lại theo đúng bản chất vật lý — tách đại lượng
                # RỜI RẠC CHÍNH XÁC ra khỏi đại lượng THAY ĐỔI ĐƯỢC:
                #   - ticks_per_rev = 44 * 90 = 3960: số nguyên, suy ra từ
                #     phần cứng (11 PPR datasheet x 4 cạnh quadrature do TIM
                #     chạy Encoder Mode TI12 x gear 90:1). Không phải số dò.
                #   - wheel_radius = 0.049: BÁN KÍNH LĂN HIỆU DỤNG, nhỏ hơn
                #     bán kính danh nghĩa 50mm ~1mm do lốp bị nén dưới tải +
                #     trượt nhẹ khi tăng tốc. Đây mới là đại lượng cần hiệu
                #     chuẩn thực nghiệm.
                #
                # ĐO THỰC NGHIỆM (2026-09-05, bánh chạm đất, chạy thẳng 8s
                # @0.2m/s): delta tick trung bình 2 bánh = 21864 (lệch trái/
                # phải chỉ 0.58%), quãng đường đo được ~1.70m (±3cm, đo áng
                # chừng vì không có thước chuẩn).
                #   -> ticks_per_rev suy ra = 3970..4113, bao trọn giá trị lý
                #      thuyết 3960 -> xác nhận encoder hoàn toàn bình thường.
                #   -> chốt 3960, suy ra bán kính lăn hiệu dụng = 49.0mm.
                # ⚠️ Mới đo 1 LẦN bằng ước lượng, chưa dùng thước chuẩn. Cần
                # đo lại vài lần ở quãng >2m khi có thước để tinh chỉnh
                # wheel_radius (chỉ cần sửa MỖI số này, ticks_per_rev giữ
                # nguyên 3960 vì là hằng số phần cứng).
                'wheel_radius':    0.049,
                'wheel_base':      0.21,
                'encoder_ppr':     44.0,   # 11 PPR x 4 cạnh quadrature (TI12)
                'gear_ratio':      90.0,
                'publish_rate_hz': 20.0,
                # Bù lệch tâm servo lái — HIỆU CHUẨN LẠI 2026-09-05 (giá trị cũ
                # -0.06 từ 2026-07-03 đã lỗi thời: sau đó đã ĐỔI SERVO sang
                # ID=1 và đấu lại cơ khí, nên lệch tâm khác hẳn).
                #
                # Cách đo (không suy luận, đo trực tiếp): cho xe chạy thẳng
                # (angular.z=0) quãng 1.08m, đo LỆCH NGANG thực tế bằng mắt/
                # thước = 21cm sang TRÁI. Coi quỹ đạo là cung tròn:
                #     s = R*theta ; y = R*(1-cos theta)
                #   -> theta = 22.57 deg, R = 2.741 m
                #   -> góc lái THẬT = atan(L/R) = atan(0.21/2.741) = +4.38 deg
                # Trong khi góc lái LỆNH lúc đó chỉ là -0.30 deg
                #   -> LỆCH TÂM CƠ KHÍ = 4.38 - (-0.30) = 4.68 deg
                #      (servo/tay đòn lắp không canh giữa — code không thể tự
                #       biết, bắt buộc đo thực tế)
                #
                # Muốn bánh thật ở 0 deg thì phải RA LỆNH -4.68 deg. Giải ngược
                # qua công thức firmware (ackermann.c: steer = ang*30 + 1.5):
                #     angular_trimmed = (-4.68 - 1.5) / 30 = -0.206
                #
                # ⚠️ Còn 2 chỗ trim cùng lúc (firmware ACK_STEER_TRIM_DEG=+1.5
                # và tham số này) — nên gộp về 1 chỗ. Sửa firmware cần nạp lại
                # từ máy Windows nên tạm giữ nguyên; số -0.206 đã tính CẢ phần
                # +1.5 của firmware nên hệ thống vẫn đi thẳng đúng.
                'steering_trim_angular_z': -0.206,
                # Dấu encoder — ĐO THỰC NGHIỆM trên Jetson (2026-09-05) với
                # wiring DRV8871 hiện tại: gửi lệnh tiến, quan sát $ODO thấy
                # enc_l chạy ÂM (0 -> -4543) còn enc_r chạy DƯƠNG (0 -> +4518),
                # độ lớn khớp nhau -> bánh trái phải nhân -1 để "tick tăng =
                # lăn tiến". Nếu KHÔNG bù, d=(dl+dr)/2 triệt tiêu về ~0 và
                # /odom đứng yên dù xe chạy thật.
                # ⚠️ ĐO LẠI sau MỖI lần đấu lại dây motor/encoder — quy ước dấu
                # không cố định qua các lần rewire (bài học lặp lại nhiều lần).
                'left_encoder_sign':  -1.0,
                'right_encoder_sign':  1.0,
            }],
        ),
    ])
