import sensor, image, time, os, tf, uos, gc
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
net = None
labels = None
try:
	net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
	print(e)
	raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
try:
	labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
	raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
clock = time.clock()
while(True):
	clock.tick()
	img = sensor.snapshot()
	for obj in net.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
		print("**********\nPredictions at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
		img.draw_rectangle(obj.rect())
		predictions_list = list(zip(labels, obj.output()))
		for i in range(len(predictions_list)):
			print("%s = %f" % (predictions_list[i][0], predictions_list[i][1]))
	print(clock.fps(), "fps")