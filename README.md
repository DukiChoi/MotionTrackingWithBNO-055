# MakeGraphUsingTinyEKFDLL + Motion Tracking With BNO-055

This uses DLL with python ctypes module to draw a 3D graph.
DLL is compiled by MS Visual Studio and the cpp file before the compilation includes the TinyEKF codes.
You can check the original TinyEKF code at https://github.com/simondlevy/TinyEKF  
I used the TinyEKF to use Extended Kalman Filter while deriving the position of a imu sensor in 3-axis from 3 types of data(accelerometer-a,angular velocity-w, geomagnetic field-h) the sensor is measuring.
Python file(graph_maker.py) includes how to use ctypes module in order to use the function in DLL file.
Graph_maker.py first makes 3 arrays(a, w, h), after reading txt files, which will be the parameters for the function I call later.
After the function call it gets an array s which is the position of the sensor, and draws a 3D graph expressing the position.


Later I adopted Arduino and IMU sensors (BNO-055) to sense 9-axis data in real-time.  In the Arduino code I used a high-pass filter to remove the drift caused when calculating Velocity and Position.
Another Python code(graph_maker_realtime.py) calculates the orientation getting the dot-product of the rotation matrix and the dots (3 dots are representing one axis, so there are 3 of (3,3) np arrays) that represent each axis(x,y,z).
And it make it possible to show the 3D graph in real-time.
![Calibration](https://github.com/DukiChoi/MakeGraphUsingTinyEKFDLL/assets/77292270/716f9f78-cfb5-42b3-9f81-e4592cef6cef)
![Drawing Circle](https://github.com/DukiChoi/MakeGraphUsingTinyEKFDLL/assets/77292270/57d0deff-4fa4-40f8-95e2-6b4349eedd99)

https://github.com/DukiChoi/MakeGraphUsingTinyEKFDLL/assets/77292270/4cb8e434-2782-4da7-b767-0f3524c3f8c8
