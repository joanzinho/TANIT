OPTICAL ODOMETRY FOR RASPBERRY AND ITS CAMERA MODULE
Cyril Rouvière


This project is a copy from RaspiVid (see RaspiCamDocs.odt) initially made to grab pictures and video from the Camera Module of Raspberry Pi.

Unfortunately, it does not include any V4L driver in order to send data to OpenCV. That is why Pierre Raufast modified it to transfert pictures from framebuffer to OpenCV :
http://thinkrpi.wordpress.com/2013/05/22/opencv-and-camera-board-csi

I added some lines to exploit data for an optical odometry intended to Eurathlon2013 project. All interresting stuff is in camcv.cpp.

Please ensure you have updated your Raspberry before using this code (August 2013).