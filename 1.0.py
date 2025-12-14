import time
import image
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
#from BSP import K230uart
# 组合多个阈值条件
WHITE_LINE_THRESHOLD = [
    (0, 25, -128, 127, -3, 127),  # 白色阈值
    (0, 0, 0, 0, 0, 0)     # 红色阈值
]
# 分辨率
pic_x = 320
pic_y = 240

# ROI区域设置
rois = [
    (0, 160, 320, 80),
    (0, 80, 320, 80),
    (0, 0, 320, 80)
]

def main():
    try:
        Display.init(Display.VIRT, width=pic_x, height=pic_y, to_ide=True)
        MediaManager.init()

        sensor = Sensor(id=2, width=pic_x, height=pic_y)
        sensor.reset()
        sensor.set_hmirror(False)
        sensor.set_framesize(width=pic_x, height=pic_y)
        sensor.set_pixformat(Sensor.RGB565)
        sensor.set_vflip(0)
        sensor.run()

        clock = time.clock()

        while True:
            clock.tick()
            img = sensor.snapshot()
            if img is None:
                continue

            processed_img = img.copy()
            processed_img.binary(WHITE_LINE_THRESHOLD)

            largest_blob = None
            largest2_blob = None
            largest3_blob = None

            # 第一个ROI区域
            blobs = processed_img.find_blobs(WHITE_LINE_THRESHOLD, roi=rois[0], merge=True)
            if blobs:
                largest_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_circle(largest_blob.cx(), largest_blob.cy(), 3, color=(255, 0, 0))
                img.draw_rectangle(rois[0], color=(255, 0, 0), thickness=1)

            # 第二个ROI区域
            blobs = processed_img.find_blobs(WHITE_LINE_THRESHOLD, roi=rois[1], merge=True)
            if blobs:
                largest2_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_circle(largest2_blob.cx(), largest2_blob.cy(), 3, color=(0, 255, 0))
                img.draw_rectangle(rois[1], color=(0, 255, 0), thickness=1)
                if largest_blob:
                    img.draw_line((largest_blob.cx(), largest_blob.cy(),
                                 largest2_blob.cx(), largest2_blob.cy()), color=(255, 255, 0))

            # 第三个ROI区域
            blobs = processed_img.find_blobs(WHITE_LINE_THRESHOLD, roi=rois[2], merge=True)
            if blobs:
                largest3_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_circle(largest3_blob.cx(), largest3_blob.cy(), 3, color=(0, 0, 255))
                img.draw_rectangle(rois[2], color=(0, 0, 255), thickness=1)
                if largest2_blob:
                    img.draw_line((largest2_blob.cx(), largest2_blob.cy(),
                                 largest3_blob.cx(), largest3_blob.cy()), color=(255, 255, 0))

            pianyi = 0
            if largest2_blob:
                pianyi = largest2_blob.cx() - (pic_x // 2)

            print("偏移: %f" % pianyi)

            Display.show_image(img)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("发生异常:", e)
    finally:
        if 'sensor' in locals():
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
