import tensorflow as tf
from tensorflow import keras
import RPi.GPIO as GPIO
import cv2
import os
import numpy as np
import time
import picamera
import signal
import atexit
from PIL import Image
import picamera.array
def dc_se_init(dcpin,servopin):
    atexit.register(GPIO.cleanup)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servopin,GPIO.OUT,initial=False)
    GPIO.setup(dcpin,GPIO.OUT,initial=False)
def dc_start(dc):
    d=GPIO.PWM(dc,20)
    d.start(0)
    return d
def se_start(servo):
    p=GPIO.PWM(servo,50)
    p.start(0)
    return p
def main():
    model = tf.saved_model.load('model_savedmodel')
    dc_se_init(27,17)
    d_c=dc_start(27)
    s_e=se_start(17)
    d_c.ChangeDutyCycle(30)
    with picamera.PiCamera() as camera:
        camera.resolution=(200,66)
        camera.framerate=30
        camera.start_preview()
        time.sleep(2)
        while True:
            camera.capture_sequence(['image1.jpg'],use_video_port=True)
            im=Image.open("/home/pi/Desktop/image1.jpg")
            print(im.mode)
            img = np.array(im)
            print(img.shape)
            imputs=tf.convert_to_tensor(img,tf.float32)
            pred=model(tf.reshape(imputs,[1,66,200,3]))
            print(pred)
            s_e.ChangeDutyCycle(abs(pred))
            
if __name__=='__main__':
    main()