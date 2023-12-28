# WebCamNoFaceUDP
The idea of this program is to have a streaming buffer being UDP sent from one PC to another to then be captured for live streaming.
When a face is detected the buffer is cleared and nothing is sent until no face is detected.
DO NOT USE THIS, IT WAS A 2 HR PROJECT AND THE DETECTION IS BAD AND PREFORMANCE IS BAD.

This was a short few hour concept done for fun.
I included the .vscode folder to make things very noob frendly? idk
The opencv path is hard coded. I have it installed / set to use c:/opencv

# Issues
-Actual FPS of retrieved webcam frames is about 2 to 6 fps...
-Hence, entier frame delay calculation is off so the real delay until UDP transmit is wrong
-Detection code is default models from opencv so it's not the best.
-Not multi threaded

# ToDo
-Better detection model
-Better webcam (DirectShow (Windows) or V4L2 (Linux) or Librealsense)
-Multi-Thread / GPU / Cuda / Vulkan
-Camera settings (Exposure, gain, resolution)
-USB 3.0 camera
