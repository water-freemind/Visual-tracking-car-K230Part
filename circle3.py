import time
import os
import sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA
from machine import Pin

# --- 基础参数 ---
sensor = None  # 摄像头实例
servo_angle = 0.0  # 舵机当前角度
RECORD_INTERVAL = 1  # 图像录制间隔
IMAGE_QUALITY = 85  # 图像保存质量
TRACK_THRESHOLD = [(42, 80, -26, 3, -55, 18)]  # 赛道二值化阈值
ROI = (0, 80, 320, 100)  # 循迹感兴趣区域
rec_roi = (0,0,320,200)  # 预留ROI
TRACK_WIDTH = 100  # 预设轨道宽度（像素）

# --- 控制参数 ---
MAX_STEERING_ANGLE = 90.0  # 舵机最大转向角度
MAX_PIXEL_ERROR = 160.0    # 最大像素偏差
Kp = 1.0  # PD控制比例系数
Kd = 0.3  # PD控制微分系数
in_cirle = 73500  # 进入环岛的里程数阈值
out_circle = 8000 # 驶出环岛的里程差阈值

# 正六边形环岛出口检测参数
CIRCLE_PERIMETER = 9000  # 正六边形环岛的周长
STRAIGHT_FRAME_THRESHOLD = 3  # 连续3帧检测到直线
straight_frame_count = 0  # 直线帧计数

# --- 环岛控制参数 ---
current_distance = 0  # 距离数据
status = 0  # 状态标识
num = 0  # 里程数
circle = 0  # 环岛模式标识
normal = 1  # 正常循迹模式标识
car_state = normal  # 初始状态
status_flag = True  # 环岛首次转向标志
circle_mode = False # 环岛模式标志
num_tag = True  # 记录进入环岛时里程数的标志
last_flag = True # 环岛驶出时转向标志
uart_buffer = b''  # 串口接收缓冲区
num_count = 0  # 进入环岛时的里程数记录

# --- 卡片检测相关参数 ---
CARD_DETECTION_ROI = (0, 0, 320, 240)  # 卡片检测区域
CARD_THRESHOLD = [(0, 180, -30, 30, 0, 50)]     # 黑色卡片阈值(HSV)
MIN_AREA = 10000  # 最小卡片面积
MAX_AREA = 50000  # 最大卡片面积
DETECTION_DELAY = 3  # 连续检测稳定次数
card_detection_state = 0  # 0:未开始,1:检测圈数,2:检测环岛数,3:完成
required_laps = 1  # 要求的圈数
required_circles = 1  # 要求的环岛次数
current_lap = 0  # 当前圈数
current_circle = 0  # 当前环岛数
circle_completed = False  # 环岛完成标志
card_detection_count_buffer = []

# --- 断轨处理新增参数 ---
history_errors = []  # 历史误差记录
HISTORY_LEN = 5  # 历史记录长度

# --- 辅助函数 ---
def check_sd_card():
    """检查存储路径 /data/ 目录是否可用"""
    try:
        os.listdir("/data")
        print("存储路径 /data/ 检测成功!")
        return True
    except OSError:
        print("错误: 存储路径 /data/ 未找到或未正确格式化。")
        return False

def find_edges_on_line(binary_img, scan_y, roi_x, roi_width, img_center_x):
    """在指定的 scan_y 高度上寻找边缘（放宽搜索范围）"""
    left_edge_x = None
    right_edge_x = None

    # 从ROI最左边向右扫描（放宽至中心右侧50像素）
    for x in range(roi_x, img_center_x + 50):
        if binary_img.get_pixel(x, scan_y)[0] > 0:
            left_edge_x = x
            break

    # 从ROI最右边向左扫描（放宽至中心左侧50像素）
    for x in range(roi_x + roi_width - 1, img_center_x - 50, -1):
        if binary_img.get_pixel(x, scan_y)[0] > 0:
            right_edge_x = x
            break

    return (left_edge_x, right_edge_x)

def line_following_control(binary_img, img_width, img):
    """优化后的循迹控制函数（支持多扫描线和断轨处理）"""
    global history_errors, HISTORY_LEN, TRACK_WIDTH

    # 增加扫描线数量（5条均匀分布）
    scan_ys = [ROI[1] + int(ROI[3] * p) for p in [0.2, 0.4, 0.6, 0.8, 1.0]]
    img_center_x = img_width // 2
    valid_centers = []  # 收集有效扫描线的轨道中心

    # 多线扫描获取有效中心
    for scan_y in scan_ys:
        left, right = find_edges_on_line(binary_img, scan_y, ROI[0], ROI[2], img_center_x)
        if left is not None and right is not None:
            valid_centers.append((left + right) // 2)
            # 可视化扫描线
            img.draw_line(ROI[0], scan_y, 319, scan_y, color=(0, 255, 0))
            img.draw_circle(left, scan_y, 3, color=(0, 0, 255))
            img.draw_circle(right, scan_y, 3, color=(0, 0, 255))

    # 处理有效中心
    if valid_centers:
        avg_center = sum(valid_centers) // len(valid_centers)
        error = avg_center - img_center_x

        # 更新历史误差
        history_errors.append(error)
        if len(history_errors) > HISTORY_LEN:
            history_errors.pop(0)

        # 计算微分误差（使用最近两次误差）
        error_diff = history_errors[-1] - history_errors[-2] if len(history_errors) >= 2 else 0
        final_error = Kp * error + Kd * error_diff

        steering_angle = (final_error / MAX_PIXEL_ERROR) * MAX_STEERING_ANGLE
        steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))
        return steering_angle, True

    # 单边缘处理
    else:
        # 尝试从最后一条扫描线获取单边缘
        last_scan_y = scan_ys[-1]
        left, right = find_edges_on_line(binary_img, last_scan_y, ROI[0], ROI[2], img_center_x)

        if left is not None:
            # 仅左边缘有效，估算中心
            estimated_center = left + TRACK_WIDTH // 2
            error = estimated_center - img_center_x
        elif right is not None:
            # 仅右边缘有效，估算中心
            estimated_center = right - TRACK_WIDTH // 2
            error = estimated_center - img_center_x
        else:
            # 完全丢线，使用历史预测
            if history_errors:
                # 基于历史趋势预测
                error_rate = history_errors[-1] - history_errors[-2] if len(history_errors)>=2 else 0
                predicted_error = history_errors[-1] + error_rate * 0.5
                error = predicted_error
            else:
                return servo_angle, False

        # 更新历史误差
        history_errors.append(error)
        if len(history_errors) > HISTORY_LEN:
            history_errors.pop(0)

        steering_angle = (error / MAX_PIXEL_ERROR) * MAX_STEERING_ANGLE
        steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))
        return steering_angle, len(history_errors) > 0  # 有历史时视为半有效

def count_blob_vertices(blob):
    """通过色块的边界框和形状特征计算顶点数量（近似）"""
    x, y, w, h = blob.rect()
    aspect_ratio = float(w) / h if h != 0 else 0

    # 使用K230 API可用的方法进行近似判断
    area = w * h
    pixel_count = blob.pixels()

    # 计算填充度
    density = pixel_count / area if area > 0 else 0

    # 基于长宽比和填充度的简单判断
    if aspect_ratio > 5:  # 很长的形状，可能是线段
        return 2
    elif 1.1 < aspect_ratio < 2.0 and density > 0.8:  # 接近正方形且填充度高
        return 4
    elif density < 0.7:  # 填充度低，可能是三角形
        return 3
    else:
        return 0  # 无法确定

def detect_card_corners(img):
    """检测卡片的角点数量，返回2(线段)、3(三角形)或4(正方形)"""
    card_img = img.copy()
    # 假设CARD_DETECTION_ROI和CARD_THRESHOLD已定义
    card_roi = CARD_DETECTION_ROI
    card_img = card_img.crop(card_roi)
    binary1 = card_img.binary(CARD_THRESHOLD)
    binary1.erode(2)
    binary1.dilate(2)

    # 寻找色块
    blobs = binary1.find_blobs(CARD_THRESHOLD, merge=True)
    if blobs:
        # 选择最大的blob
        largest_blob = max(blobs, key=lambda b: b.pixels())
        vertices = count_blob_vertices(largest_blob)
        return vertices
    else:
        return 0
# 寻找色块（替代轮廓检测）
    blobs = binary1.find_blobs(
        CARD_THRESHOLD,  # 补充必需的阈值参数（位置参数）
        area_threshold=MIN_AREA,  # 用关键字传递面积过滤参数
        pixels_threshold=MAX_AREA  # 用关键字传递像素数过滤参数（可选）
     )

    max_area = 0
    best_blob = None

    for blob in blobs:
        area = blob.area()
        if MIN_AREA < area < MAX_AREA and area > max_area:
            max_area = area
            best_blob = blob

    if best_blob:
        vertices = count_blob_vertices(best_blob)
        img.draw_rectangle(card_roi, color=(255, 255, 0))
        if vertices >= 2:
            x, y, w, h = best_blob.rect()
            x += card_roi[0]
            y += card_roi[1]
            img.draw_rectangle((x, y, w, h), color=(0, 255, 255))
            cx, cy = best_blob.cx() + card_roi[0], best_blob.cy() + card_roi[1]
            img.draw_cross(cx, cy, color=(0, 255, 0))
            return vertices

    return 0  # 未检测到有效形状

def process_card_detection(img):
    """处理卡片检测流程，返回是否完成检测"""
    global card_detection_state, required_laps, required_circles, DETECTION_DELAY
    global card_detection_count_buffer

    corners = detect_card_corners(img)

    if corners > 0:
        card_detection_count_buffer.append(corners)
        if len(card_detection_count_buffer) > DETECTION_DELAY:
            card_detection_count_buffer.pop(0)

    # 状态0: 等待开始检测
    if card_detection_state == 0:
        img.draw_string_advanced(30, 50,20, "按按键开始检测", (0, 255, 0))
        img.draw_string_advanced(10, 80,15,"首次:圈数(2-线段,3-三角,4-正方)",  (0, 255, 0))
        img.draw_string_advanced(10, 100,15, "第二次:环岛数",  (0, 255, 0))
        if button.value() == 1:
            card_detection_state = 1
            card_detection_count_buffer = []
            time.sleep_ms(500)

    # 状态1: 检测圈数卡片
    elif card_detection_state == 1:
        img.draw_string_advanced(50, 30,20, "检测圈数卡片中",  (255, 0, 0))
        if len(card_detection_count_buffer) == DETECTION_DELAY:
            result = max(set(card_detection_count_buffer), key=card_detection_count_buffer.count)
            if result in [2, 3, 4]:
                required_laps = result
                img.draw_string_advanced(100, 80, 30,f"圈数: {required_laps}",  (0, 255, 0))
                img.draw_string_advanced(20, 120, 15,"按按键确认",  (0, 255, 0))
                if button.value() == 1:
                    card_detection_state = 2
                    card_detection_count_buffer = []
                    time.sleep_ms(500)

    # 状态2: 检测环岛数卡片
    elif card_detection_state == 2:
        img.draw_string_advanced(30, 30,20, "检测环岛数卡片中",  (255, 0, 0))
        if len(card_detection_count_buffer) == DETECTION_DELAY:
            result = max(set(card_detection_count_buffer), key=card_detection_count_buffer.count)
            if result in [2, 3, 4]:
                required_circles = result
                img.draw_string_advanced(100, 80,30, f"环岛数: {required_circles}",  (0, 255, 0))
                img.draw_string_advanced(20, 120,15, "按按键开始", (0, 255, 0))
                if button.value() == 1:
                    card_detection_state = 3
                    card_detection_count_buffer = []
                    time.sleep_ms(500)

    return card_detection_state == 3

try:
    fpioa = FPIOA()
    # 初始化按键
    fpioa.set_function(53, FPIOA.GPIO53)
    button = Pin(53, Pin.IN, Pin.PULL_DOWN)
    # 初始化LED
    fpioa.set_function(62, FPIOA.GPIO62)
    LED_R = Pin(62, Pin.OUT)
    LED_R.high()

    print("系统准备就绪，请按下按键以启动程序...")
    while button.value() == 0:
        LED_R.low()

    LED_R.high()
    print("按键已按下，正在启动主程序...")

    # 硬件初始化
    fpioa.set_function(50, FPIOA.UART3_TXD)
    fpioa.set_function(51, FPIOA.UART3_RXD)
    uart = UART(UART.UART3, baudrate=115200)
    uartFlag = True

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=320, height=240)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701)
    MediaManager.init()

    RECORDING_ENABLED = True
    folder_name = ""
    if not check_sd_card():
        RECORDING_ENABLED = False
        print("录像功能已禁用。")
    else:
        current_time = time.localtime()
        folder_name = f"/data/run_{current_time[0]:04d}{current_time[1]:02d}{current_time[2]:02d}_{current_time[3]:02d}{current_time[4]:02d}{current_time[5]:02d}_{time.ticks_ms()}"
        try:
            os.mkdir(folder_name)
            print(f"成功创建文件夹: {folder_name}")
        except OSError:
            print(f"文件夹 {folder_name} 已存在。")

    sensor.run()
    clock = time.clock()
    frame_counter = 0
    saved_frame_counter = 0

    # 记录一圈的里程数（用于圈数计数）
    lap_distance = 0
    last_num = 0
    lap_start_num = 0
    lap_started = False

    while True:
        clock.tick()
        os.exitpoint()

        # 串口数据接收处理
        if uart.any():
            read_data = uart.read()
            if read_data:
                uart_buffer += read_data

        # 处理缓冲区中所有完整的行
        while b'\n' in uart_buffer:
            line_bytes, uart_buffer = uart_buffer.split(b'\n', 1)
            try:
                line_str = line_bytes.decode('utf-8').strip()
                # 处理距离数据
                if line_str.startswith("dist="):
                    parts = line_str.split('=')
                    if len(parts) == 2:
                        value_str = parts[1].strip()
                        if value_str.isdigit():
                            current_distance = int(value_str)
                        else:
                            print(f"无效的距离数值: {value_str}")

                # 处理状态数据
                if line_str.startswith("status="):
                    parts = line_str.split('=')
                    if len(parts) == 2:
                        value_str = parts[1].strip()
                        if value_str.isdigit():
                            status = int(value_str)
                        else:
                            print(f"无效的状态数值: {value_str}")

                # 处理里程数数据
                if line_str.startswith("num="):
                    parts = line_str.split('=')
                    if len(parts) == 2:
                        value_str = parts[1].strip()
                        if value_str.isdigit():
                            num = int(value_str)

                            # 计算圈数
                            if lap_started and num < lap_start_num:  # 里程数溢出，说明完成一圈
                                current_lap += 1
                                lap_start_num = num
                                print(f"完成第 {current_lap} 圈")
                            elif not lap_started:
                                lap_start_num = num
                                lap_started = True
                        else:
                            print(f"无效的里程数值: {value_str}")
            except UnicodeError:
                print("串口数据解码失败")
            except Exception as e:
                print(f"处理串口数据时出错: {e}")

        # 图像处理（增加断轨修复）
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        binary_img = img.binary(TRACK_THRESHOLD)
        # 形态学操作修复短断轨：先膨胀连接缺口，再腐蚀恢复宽度
        binary_img.dilate(2)
        binary_img.erode(2)
        # 保留原有开运算去除噪声
        kernel_size = 1
        binary_img.open(kernel_size)

        # 显示当前里程、状态信息
        img.draw_string_advanced(20, 200,20, f"dist: {current_distance}",  (255, 0, 0))
        img.draw_string_advanced(200, 220,20, f"status{status}",  (0, 255, 0))
        img.draw_string_advanced(100, 220,20, f"num{num}",  (0, 255, 0))

        # 显示圈数和环岛数信息
        img.draw_string_advanced(20, 170,15, f"圈数: {current_lap}/{required_laps}",  (255, 255, 0))
        img.draw_string_advanced(150, 170,15, f"环岛: {current_circle}/{required_circles}",  (255, 255, 0))

        # 先进行卡片检测，完成后再开始循迹
        if card_detection_state < 3:
            detection_completed = process_card_detection(img)
            # 未完成检测时不进行循迹控制
            uart.write("aspeed=0\r\n")
            uart.write("bspeed=0\r\n")
        else:
            # 检查是否完成所有任务
            if current_lap >= required_laps and current_circle >= required_circles:
                img.draw_string_advanced(50, 100,30,  "任务完成!", (0, 255, 0))
                uart.write("aspeed=0\r\n")
                uart.write("bspeed=0\r\n")
                time.sleep(2)
                continue

            # 环岛进入判断
            if num > in_cirle and not circle_completed and current_circle < required_circles:
                if num_tag == True:
                    num_count = num  # 记录进入环岛时的里程数
                    num_tag = False
                    straight_frame_count = 0  # 重置直线计数
                car_state = circle
                circle_completed = False  # 标记正在进行环岛

            # 统一的循迹控制
            if car_state == normal:
                servo_angle, line_detected = line_following_control(binary_img, img.width(),img)

            # 环岛模式控制
            if car_state == circle:
                # 1. 环岛首次强制转向
                if status_flag:
                    servo_angle = -45.0  # 强制左转
                    uart.write(f"k230servo={servo_angle}\r\n")
                    time.sleep_ms(500)
                    status_flag = False

                # 2. 执行循迹控制
                servo_angle, line_detected = line_following_control(binary_img, img.width(),img)

                # 3. 驶出前强制转向
                if num - num_count >= out_circle and last_flag == True:
                    last_flag = False
                    servo_angle = -45.0  # 强制左转调整方向
                    uart.write(f"k230servo={servo_angle}\r\n")
                    time.sleep_ms(500)

                # 4. 出口检测
                if num - num_count >= CIRCLE_PERIMETER * 0.9:  # 里程数达到周长90%
                    scan_y_near = ROI[1] + int(ROI[3] * 0.8)
                    scan_y_far = ROI[1] + int(ROI[3] * 0.2)
                    img_center_x = img.width() // 2

                    # 复用边缘检测逻辑
                    left_near, right_near = find_edges_on_line(binary_img, scan_y_near, ROI[0], ROI[2], img_center_x)
                    left_far, right_far = find_edges_on_line(binary_img, scan_y_far, ROI[0], ROI[2], img_center_x)

                    if left_near and right_near and left_far and right_far:
                        center_near = (left_near + right_near) // 2
                        center_far = (left_far + right_far) // 2
                        error_diff = abs(center_far - center_near)

                        # 连续检测到直线，确认出口
                        if error_diff < 10:
                            straight_frame_count += 1
                        else:
                            straight_frame_count = 0

                        if straight_frame_count >= STRAIGHT_FRAME_THRESHOLD:
                            print("检测到正六边形环岛出口！切换回正常循迹")
                            car_state = normal
                            # 重置环岛相关标志
                            num_tag = True
                            last_flag = True
                            status_flag = True
                            straight_frame_count = 0
                            current_circle += 1  # 记录完成的环岛数
                            circle_completed = True  # 标记该环岛已完成

                # 可视化环岛信息
                img.draw_string_advanced(0, 230,15, f"straight: {straight_frame_count}/{STRAIGHT_FRAME_THRESHOLD}",  (255, 0, 0))
                img.draw_string_advanced(150, 230,15,  f"circle_dist: {num - num_count}", (0, 255, 255))

            # 丢线提示
            if not line_detected:
                if num <= 76000:
                    img.draw_string_advanced(10, 10, 30,"Line lost!",  (255, 0, 0))
            else:
                img.draw_string_advanced(10, 10,30, f"angle: {servo_angle:.1f}",  (255, 0, 0))

            # 舵机指令发送
            uart.write(f"k230servo={servo_angle}\r\n")

            # 状态显示
            img.draw_string_advanced(10, 35, 40, f"FPS: {clock.fps():.2f}", (0, 255, 0))

            # 初始化速度设置
            if uartFlag:
                uart.write("aspeed=65\r\n")
                uart.write("bspeed=65\r\n")
                uart.write("start\r\n")
                uartFlag = False

            # 录制功能
            frame_counter += 1
            if RECORDING_ENABLED and (frame_counter % RECORD_INTERVAL == 0):
                saved_frame_counter += 1
                filename = f"{folder_name}/frame_{saved_frame_counter:06d}.jpg"
                img.save(filename, quality=IMAGE_QUALITY)

        # 屏幕显示
        img.compressed_for_ide()
        Display.show_image(img)

except KeyboardInterrupt as e:
    print("User Stop", e)
except BaseException as e:
    print(f"error {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
