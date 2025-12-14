import time
import os
import sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA
from machine import Pin
from libs.K230uart import SerialProtocol
# --- 基础参数 ---
sensor = None  # 摄像头实例
dis_err = 0.0  # 中心偏差值
RECORD_INTERVAL = 1  # 图像录制间隔
IMAGE_QUALITY = 85  # 图像保存质量
TRACK_THRESHOLD = [(42, 80, -26, 3, -55, 18)]  # 赛道二值化阈值(42, 80, -26, 3, -55, 18)
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
card_detection_count_buffer = [] #卡片检测结果缓冲区域
detection_start_time = 0 #检测阶段开始的时间

# --- 断轨处理新增参数 ---
history_errors = []  # 历史误差记录
HISTORY_LEN = 5  # 历史记录长度

# 新增矩形检测函数
def detect_start_rectangle(binary_img):
    """检测起点矩形，返回是否检测到"""
    # 在指定ROI内查找色块
    rect_roi_img = binary_img.crop(RECTANGLE_ROI)
    blobs = rect_roi_img.find_blobs(TRACK_THRESHOLD, merge=True)  # 复用轨道阈值

    if not blobs:
        return False

    # 筛选符合面积和形状的色块
    for blob in blobs:
        x, y, w, h = blob.rect()
        area = blob.pixels()
        aspect_ratio = float(w) / h  # 宽高比

        # 检查面积和宽高比是否符合矩形特征
        if (RECTANGLE_MIN_AREA < area < RECTANGLE_MAX_AREA and
            RECTANGLE_ASPECT_RATIO[0] < aspect_ratio < RECTANGLE_ASPECT_RATIO[1]):
            return True
    return False

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
                return dis_err, False

        # 更新历史误差
        history_errors.append(error)
        if len(history_errors) > HISTORY_LEN:
            history_errors.pop(0)

        steering_angle = (error / MAX_PIXEL_ERROR) * MAX_STEERING_ANGLE
        steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))
        return steering_angle, len(history_errors) > 0  # 有历史时视为半有效

def count_blob_vertices(blob):
    """通过色块的形状特征计算顶点数量（近似）"""
    x, y, w, h = blob.rect()

    if h == 0 or w == 0:
        return 0        #避免出现宽高除零的错误

    aspect_ratio = float(w) / h #计算宽高比
    # 使用K230 API可用的方法进行近似判断
    area = w * h
    pixel_count = blob.pixels()

    # 计算填充度
    density = pixel_count / area if area > 0 else 0

    #print(density)
    # 基于长宽比的简单判断
    # 1. 优先判断正方形：宽高比非常接近 1.0
    # 可以根据实际卡片的规整程度微调这个范围
    if 1.2 < aspect_ratio < 1.5 and density<0.55:
        return 4  # 正方形

    # 2. 然后判断线段：宽高比非常大或非常小
    # 宽度远大于高度，或者高度远大于宽度
    if aspect_ratio < 0.8 and density > 0.8:
        return 2  # 线段

    # 3. 最后，剩下判断为三角形
    # 如果它既不是正方形，也不是线段，就认为它是三角形
    return 3  # 三角

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
    """
    处理卡片检测流程，返回是否完成检测
    加了一个超时逻辑检测
    """
    global card_detection_state, required_laps, required_circles, DETECTION_DELAY
    global card_detection_count_buffer,detection_start_time #全局计时器

    corners = detect_card_corners(img) #获取当前检测订点数

    # --- 超时逻辑核心：状态切换时记录开始时间 ---
    if card_detection_state == 1 and detection_start_time == 0:
        # 刚进入“检测圈数”状态，记录起始时间
        detection_start_time = time.ticks_ms()
    elif card_detection_state == 2 and detection_start_time == 0:
        # 刚进入“检测环岛数”状态，记录起始时间
        detection_start_time = time.ticks_ms()

    # 仅当检测到有效形状时，才向缓冲区添加结果
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
            detection_start_time = 0 # 重置计时器
            time.sleep_ms(500)

    # 状态1: 检测圈数卡片
    elif card_detection_state == 1:
        img.draw_string_advanced(50, 30,20, "检测圈数卡片中",  (255, 0, 0))

        # 检查是否超时 (5000ms = 5秒)
        if time.ticks_ms() - detection_start_time > 7500:
            img.draw_string_advanced(30, 80, 20, "超时未检测到卡片!", (255, 255, 0))
            img.draw_string_advanced(50, 110, 25, "默认圈数: 1", (255, 255, 0))
            required_laps = 1 # 超时，设置默认值
            time.sleep(1) # 延时1秒，让用户看清提示
            # 切换到下一状态
            card_detection_state = 2
            card_detection_count_buffer = []
            detection_start_time = 0 # 重置计时器
        # 正常检测逻辑
        if len(card_detection_count_buffer) == DETECTION_DELAY:
            result = max(set(card_detection_count_buffer), key=card_detection_count_buffer.count)
            if result in [2, 3, 4]:
                required_laps = result
                img.draw_string_advanced(100, 80, 30,f"圈数: {required_laps}",  (0, 255, 0))
                img.draw_string_advanced(20, 120, 15,"按按键确认",  (0, 255, 0))
                if button.value() == 1:
                    card_detection_state = 2
                    card_detection_count_buffer = []
                    detection_start_time = 0 # 重置计时器
                    time.sleep_ms(500)

    # 状态2: 检测环岛数卡片
    elif card_detection_state == 2:
        img.draw_string_advanced(30, 30,20, "检测环岛数卡片中",  (255, 0, 0))

        # 检查是否超时 (5000ms = 5秒)
        if time.ticks_ms() - detection_start_time > 7500:
            img.draw_string_advanced(30, 80, 20, "超时未检测到卡片!", (255, 255, 0))
            img.draw_string_advanced(50, 110, 25, "默认环岛数: 1", (255, 255, 0))
            required_circles = 1 # 超时，设置默认值
            time.sleep(1) # 延时1秒，让用户看清提示
            # 切换到完成状态
            card_detection_state = 3
            card_detection_count_buffer = []
        # 正常检测逻辑
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
    #串口初始化
    User_Data = [required_laps,required_circles,int(dis_err),0,status]#圈数,环岛数,偏差值,起始点,环岛标志
    protocol = SerialProtocol(uart_id=UART.UART2, baudrate=115200, tx_pin=11, rx_pin=12)

    print("准备就绪，按下按键以启动程序...")
    while button.value() == 0:
        LED_R.low()

    LED_R.high()
    print("按键已按下，正在启动主程序...")

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
        User_Data = [required_laps,required_circles,int(dis_err),0,status]#圈数,环岛数,偏差值,起始点,环岛标志
        '''
        line_str = b"dist=100\nstatus=1\nnum=5000\n"

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

                    # 圈数计数逻辑（结合矩形检测）
                    #global first_lap, rect_detected, current_lap, lap_start_num, lap_started
                    if not lap_started:
                        lap_start_num = num
                        lap_started = True
                    else:
                        # 里程数溢出（完成一圈），且非首次运行时需检测到矩形
                        if num < lap_start_num:
                            if first_lap:
                                # 首次完成一圈，无需检测矩形
                                current_lap += 1
                                first_lap = False  # 后续圈数需要检测矩形
                                lap_start_num = num
                                print(f"完成第 {current_lap} 圈（首次）")
                            else:
                                # 非首次，需检测到矩形才计数
                                if rect_detected:
                                    current_lap += 1
                                    rect_detected = False  # 重置检测状态
                                    lap_start_num = num
                                    print(f"完成第 {current_lap} 圈（检测到矩形）")
                                else:
                                    print("未检测到起点矩形，不计数")
                                    '''
        # 图像处理（增加断轨修复）
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        binary_img = img.binary(TRACK_THRESHOLD)
        '''
        # 检测起点矩形（仅非首次运行时需要）
        if not first_lap and card_detection_state >= 3:
            rect_detected = detect_start_rectangle(binary_img)
            # 可视化矩形检测区域和状态
            img.draw_rectangle(RECTANGLE_ROI, color=(255, 0, 255))  # 紫色框标记矩形ROI
            if rect_detected:
                img.draw_string_advanced(RECTANGLE_ROI[0], RECTANGLE_ROI[1]-15, 15, "Rect Found", (0, 255, 0))
            else:
                img.draw_string_advanced(RECTANGLE_ROI[0], RECTANGLE_ROI[1]-15, 15, "Looking for Rect", (255, 0, 0))
                '''
        # 形态学操作修复短断轨：先膨胀连接缺口，再腐蚀恢复宽度
        binary_img.dilate(2)
        binary_img.erode(2)
        # 保留原有开运算去除噪声
        kernel_size = 1
        binary_img.open(kernel_size)

        # 显示当前里程、状态信息
        img.draw_string_advanced(5, 220,20, f"dist: {current_distance}",  (255, 0, 0))
        img.draw_string_advanced(240, 220,20, f"status{status}",  (0, 255, 0))
        img.draw_string_advanced(150 , 220,20, f"num{num}",  (0, 255, 0))

        # 显示圈数和环岛数信息
        img.draw_string_advanced(20, 170,15, f"圈数: {current_lap}/{required_laps}",  (0, 255, 0))
        img.draw_string_advanced(240, 170,15, f"环岛: {current_circle}/{required_circles}",  (0, 255, 0))

        # 先进行卡片检测，完成后再开始循迹
        if card_detection_state < 3:
            detection_completed = process_card_detection(img)
            # 未完成检测时不进行循迹控制

        else:
            # 检查是否完成所有任务
            if current_lap >= required_laps and current_circle >= required_circles:
                img.draw_string_advanced(50, 100,30,  "任务完成!", (0, 255, 0))
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
                protocol.send(*User_Data)
                dis_err, line_detected = line_following_control(binary_img, img.width(),img)

            # 环岛模式控制
            if car_state == circle:
                # 1. 环岛首次强制转向
                if status_flag:
                    #处理方法
                    status_flag = False

                # 2. 执行循迹控制
                dis_err, line_detected = line_following_control(binary_img, img.width(),img)

                # 3. 驶出前强制转向
                if num - num_count >= out_circle and last_flag == True:
                    last_flag = False
                    #处理方法

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
                img.draw_string_advanced(10, 10,20, f"dis_err: {dis_err:.1f}",  (255, 0, 0))

            # 状态显示
            img.draw_string_advanced(10, 40, 20, f"FPS: {clock.fps():.2f}", (0, 255, 0))
            #发送串口数据
            protocol.send(*User_Data)
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
