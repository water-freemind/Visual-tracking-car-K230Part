import time
import image
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

# 阈值设置 - 灰度图
GRAYSCALE_THRESHOLD = [(0, 76)]
turnGRAYSCALE_THRESHOLD = [(51, 255)]

# 使用QVGA分辨率 320x240
pic_x = 320
pic_y = 240

# ROI区域设置
rois = [
    (0, 160, 320, 80),    # 底部区域
    (0, 80, 320, 80),     # 中部区域
    (0, 0, 320, 80)       # 顶部区域
]

def main():
    try:
        Display.init(Display.VIRT, width=pic_x, height=pic_y, to_ide=True)
        MediaManager.init()

        sensor = Sensor(id=2, width=pic_x, height=pic_y)
        sensor.reset()
        sensor.set_hmirror(False)
        sensor.set_framesize(width=pic_x, height=pic_y)
        sensor.set_pixformat(Sensor.GRAYSCALE)  # 使用灰度图节省内存
        sensor.set_vflip(0)

        sensor.run()
        clock = time.clock()

        while True:
            clock.tick()
            img = sensor.snapshot()
            if img is None:
                continue

            img.binary(turnGRAYSCALE_THRESHOLD)

            largest_blob = None
            largest2_blob = None
            largest3_blob = None

            # 第一个ROI区域
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=rois[0], merge=True)
            if blobs:
                largest_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_circle(largest_blob.cx(), largest_blob.cy(), 3)

            # 第二个ROI区域
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=rois[1], merge=True)
            if blobs:
                largest2_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_circle(largest2_blob.cx(), largest2_blob.cy(), 3)
                if largest_blob:
                    img.draw_line((largest_blob.cx(), largest_blob.cy(),
                                 largest2_blob.cx(), largest2_blob.cy()))

            # 第三个ROI区域
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=rois[2], merge=True)
            if blobs:
                largest3_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_circle(largest3_blob.cx(), largest3_blob.cy(), 3)
                if largest2_blob:
                    img.draw_line((largest2_blob.cx(), largest2_blob.cy(),
                                 largest3_blob.cx(), largest3_blob.cy()))

            pianyi = 0
            if largest2_blob:
                pianyi = largest2_blob.cx() - (pic_x // 2)

            print("偏移: %f" % pianyi)
            print("FPS: %f" % clock.fps())
            Display.show_image(img)

    except KeyboardInterrupt:
        print("用户停止程序")
    except Exception as e:
        print("发生异常:", e)
    finally:
        if 'sensor' in locals():
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
