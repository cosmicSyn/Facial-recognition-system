# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Face Detection Example
#
# This example shows off the built-in face detection feature of the OpenMV Cam.
#
# Face detection works by using the Haar Cascade feature detector on an image. A
# Haar Cascade is a series of simple area contrasts checks. For the built-in
# frontalface detector there are 25 stages of checks with each stage having
# hundreds of checks a piece. Haar Cascades run fast because later stages are
# only evaluated if previous stages pass. Additionally, your OpenMV Cam uses
# a data structure called the integral image to quickly execute each area
# contrast check in constant time (the reason for feature detection being
# grayscale only is because of the space requirement for the integral image).

from pyb import LED
from machine import Pin
import sensor, image, time, os, tf, uos, gc
# Reset sensor
sensor.reset()

# Sensor settings
sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.HQVGA)      # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 240))       # Set 240x240 window.
sensor.skip_frames(time=2000)          # Let the camera adjust.
sensor.set_contrast(3)
sensor.set_gainceiling(16)
# HQVGA and GRAYSCALE are the best for face tracking.


# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)
print(face_cascade)

# Variables
p0 = Pin('P0', Pin.OUT)
net = None
labels = None
red_led = LED(2)
green_led = LED(3)
# FPS clock

try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    print(e)
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')


clock = time.clock()

while True:
    clock.tick()
    green_led.off()
    red_led.off()
    p0.off()
    # Capture snapshot
    img = sensor.snapshot()
    objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)
    # Find objects.
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.

    if objects != []:
        time.sleep_ms(5000)
        img = sensor.snapshot()
        objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)
        if objects != []:
            img = sensor.snapshot()
            objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)
            # Draw objects
            for r in objects:
                img.draw_rectangle(r)

            # default settings just do one detection... change them to search the image...
            for obj in net.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
                print("**********\nPredictions at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
                img.draw_rectangle(obj.rect())
                # This combines the labels and confidence values into a list of tuples
                predictions_list = list(zip(labels, obj.output()))

                for i in range(len(predictions_list)):
                    print("%s = %f" % (predictions_list[i][0], predictions_list[i][1]))

                    auth_num = float(predictions_list[0][1])
                    if(auth_num>0.62):
                       green_led.on()
                       red_led.off()
                       p0.on()
                    else:
                       green_led.off()
                       red_led.on()

            time.sleep_ms(5000)



    # Print FPS.
    # Note: Actual FPS is higher, streaming the FB makes it slower.
#    print(clock.fps())
