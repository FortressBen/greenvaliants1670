import sensor, image, time, pyb
SQUARES = 4
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster (160x120 max on OpenMV-M7)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False,300)
sensor.skip_frames(time = 2000)
clock = time.clock()
Threshold = (69, 255)
indicator_green = pyb.Pin('P7', pyb.Pin.OUT_PP)
indicator_red = pyb.Pin('P9', pyb.Pin.OUT_PP)
while(True):
    clock.tick()
    img = sensor.snapshot()

    # `threshold` below should be set to a high enough value to filter out noise
    # rectangles detected in the image which have low edge magnitudes. Rectangles
    # have larger edge magnitudes the larger and more contrasty they are...

    blobs = img.find_blobs([Threshold])
    list_o_blobs = 0
    for blob in blobs:
        print(blob.pixels())
        if blob.pixels() <= 200 and blob.pixels() >= 80:
            aspect_ratio = blob.h() / blob.w()
            if aspect_ratio > 0.75 and aspect_ratio < 1.25:
                img.draw_rectangle(blob.rect(), color=(255,255,255), thickness=2)
                list_o_blobs += 1
            else:
                print(aspect_ratio)
            
    if list_o_blobs == SQUARES:
         print("empty")
         indicator_green.value(1)
         indicator_red.value(0)
    else:
        print("not empty")
        indicator_green.value(0)
        indicator_red.value(1)
    print(list_o_blobs)

    print("FPS %f" % clock.fps())
