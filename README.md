# Ros Streaming Cam
This ROS package creates a RTSP server to stream a camera video through the network. It uses Gstreamer v1.0 then, in order to compile and use this package, it has to be correctly installed. Note that also *dev* libraries are necessary. In Ubuntu open a terminal and type:

```sudo apt-get install apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-rtsp gstreamer1.0-libav gstreamer1.0-tool libgstreamer-plugins-good1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev   ``` 

The stream video is published in rtsp://{ip}:8554{url}. Open a client RTSP, like VLC, and open this url.

## Parameters

In the launch file there are a lot parameters to configure the package and tune the pipeline.

* **url:** set the url where the stream are published
* **topic:** where the camera publishes the frames (used a topic that transmits sensor_msgs:Image)
* **color:** the color format of the frames (e.g. RGB)
* **width:** width of the frames
* **height:** height of the frames
* **framerate:** the framerate assigned to the video stream
* **profile:** the quality of the video stream. There are three standard profile: *high-quality*, *medium-quality* and *low-quality*. In order to optimize better the pipeline there is profile called *customized* that consider the following three parameters used by h264 encoder.
* **subme:** subpixel motion estimation and partition decision quality: (fast - worst) 1 <= x <= 10 (slow - best)
* **quantizer:** constant quantizer or quality to apply: (high quality) 1 <= x <= 50 (low quality)
* **threads:** Number of threads used by the encoder (0 = auto)