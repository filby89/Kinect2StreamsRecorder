# Kinect2StreamsRecorder
Recording of Kinect V2 Streams at 30 fps. You need an I7 3.1 GHz (or higher) (the same is officially required for Kinect V2) and a SSD disk
to record or else your ram will explode. When testing for the first few times keep your system monitor open and if ram usage is totally
increasing stop the recording and select less streams (the most computationally expensive are the color stream due to conversion to 100 quality JPEG 
and the face HD stream). The GUI also shows the number of frames in each of the different stream buffers to give you an idea of the 
memory used. Ideally these numbers should be around 0.
## Installation
Clone this repository and build with Visual Studio 2015 for x64 architecture and release mode. Kinect V2 API is required. 
## Usage
The software uses by default 4 threads:
* One for handling the GUI the 2 face stream arrivals and the audio stream arrival.
* One for handling a multisourceframe arrival
* One for saving the color stream from the buffer that it is stored from the other thread to the disk
* One for all the other streams that are stored in buffers from the other threads to the disk

When the number of color frames stored in the buffer exceedes a threshold another thread is allocated to try and stop
the memory usage from increasing. You can also choose via the GUI to have this second thread constantly allocated.

Choose the SSD disk from the folder browser. You can also choose a name for the recording. Each different recording is 
saved in a different folder according to the timestamp it started. Upon stopping a recording, wait for the frames still in buffer to
be written to the disk.

## Data Recorded Format
When all options are selected the directory structure is as follows:

${selected_dir}/${subject}/YYY_mm_dd-hh_mm_ss/

### log.txt 
The log has info about the recorded frames and what the ideal number of frames (according to the recording time) should be.

### calib.txt 
This file stores the [depth to camera space](https://msdn.microsoft.com/en-us/library/windowspreview.kinect.coordinatemapper.getdepthframetocameraspacetable.aspx) lookup table. Also stores FocalLengthX, FocalLengthY, PrincipalPointX, PrincipalPointY, RadialDistortionSecondOrder, RadialDistortionFourthOrder and RadialDistortionSixthOrder.

### /Color 
Color frames saved in JPEG format. Additionally stored is 'colorData.csv' containing the frame counter (note it does not start from 1), the [relative time](https://docs.microsoft.com/en-us/uwp/api/Windows.Foundation.TimeSpan) 
since kinect was opened and the [unix time stamp](https://msdn.microsoft.com/en-us/library/system.datetimeoffset.tounixtimemilliseconds(v=vs.110).aspx)  of the frames. Frames are saved in format: ${relative_time}\_{counter}.jpg.

### /Depth
Depth frames saved in JPEG format. Additionally stored is 'depthData.csv' containing the frame counter (note it does not start from 1), the [relative time](https://docs.microsoft.com/en-us/uwp/api/Windows.Foundation.TimeSpan) 
since kinect was opened and the [unix time stamp](https://msdn.microsoft.com/en-us/library/system.datetimeoffset.tounixtimemilliseconds(v=vs.110).aspx), the min, and the max reliable depth in millimeters of the frames. Frames are saved in format: ${relative_time}\_{counter}.txt. 

To reconstruct a depth image you can use 
```python
import numpy as np
import os
import matplotlib.pyplot as plt
from PIL import Image

file = "depth.txt"
arr = np.fromfile(file, np.int16)
plt.imshow(arr.reshape((512,424), order='F'), cmap='gray')
plt.show()
```

### /BodyIndex
* Body Index frames saved in binary format. Additionally stored is 'bodyIndexData.csv' containing the frame counter (note it does not start from 1), the [relative time](https://docs.microsoft.com/en-us/uwp/api/Windows.Foundation.TimeSpan) 
since kinect was opened and the [unix time stamp](https://msdn.microsoft.com/en-us/library/system.datetimeoffset.tounixtimemilliseconds(v=vs.110).aspx), the min, and the max reliable depth in millimeters of the frames. Frames are saved in format: ${relative_time}\_{counter}.txt. 

To reconstruct a body index image you can use 
```python
import numpy as np
import os
import matplotlib.pyplot as plt
from PIL import Image

file = "depth.txt"
arr = np.fromfile(file, np.uint8)
plt.imshow(arr.reshape((512,424), order='F'), cmap='gray')
plt.show()
```
### /Skeleton
The skeleton is saved in csv format with semicolon separators. For each frame all 6 bodies are saved detected or not (which results in lot of useless rows). Each row contains the following data:
* 0: Counter of associated depth frame
* 1: Relative time
* 2: Unix timestamp
* 3: Joint Name
* 4: Joint Tracking State
* 5-7: 3D joint position
* 8-9: Joint position in Depth Space
* 10-11: Joint position in Color Space
* 12-15: Joint Orientation (quaternion)

... repeated for all 25 joints ...

* 329: ClippedEdges 
* 330: HandLeftState 
* 331: HandLeftConfidence
* 332: HandRightState 
* 333: HandRightConfidence
* 334: If body is tracked
* 335: Joint Count
* 336: Tracking ID of body
* 337-338: Lean
* 339: Lean Tracking State

### /Face
The face is saved in csv format with semicolon separators. Each row is associated to a detected face.
Each row contains the following data:
* 0: Relative time of the associated body frame
* 1: Relative time of the associated color frame
* 2: Unix timestamp
* 3: Tracking ID of associated body
* 4: If the tracking is valid
* 5-8: Face Bounding box in Color Space (Bottom, Left, Top, Right)
* 9-12: Face Bounding box in Depth Space (Bottom, Left, Top, Right)
* 13-28: Face Properties (ith column has property name, ith + 1 column has property value) 
* 29-43: Facial landmarks in Color Space (ith column has landmark name, ith+1 and ith+2 have X and Y positions)
* 44-58: Facial landmarks in Color Space (ith column has landmark name, ith+1 and ith+2 have X and Y positions)
* 59-62: Facial Rotation quaternion
* 63-65: Yaw Pitch Roll (calculated from the above quaternion)

### /FaceHD
The hd face is saved in csv format with semicolon separators. Each row is associated to a detected face.
Each row contains the following data:
* 0: Relative time of the hd face frame
* 1: Relative time of the associated depth frame
* 2: Unix timestamp
* 3: Tracking ID of associated body
* 4: If the tracking is valid
* 5-9433: All 1347 vertices of the hd face. For each vertice the following are saved: (3D.X, 3D.Y, 3D.Z, DepthX, DepthY, ColorX, ColorY)
* 9434-9467: Face shape animations (ith column has animation name, ith+1 has value)
* 9468-9471: Facial Rotation quaternion
* 9472-9474: Yaw Pitch Roll (calculated from the above quaternion)
* 9475-9481: (3D.X, 3D.Y, 3D.Z, DepthX, DepthY, ColorX, ColorY)
* 9482: Quality

### /Audio
Audio is saved in .raw format while the beam information for each subframe is saved in .csv. To convert it to wav you can use sox:
```
$ sox -r 16000 -c 1 -b 32 -e floating_point audio.raw audio.wav
```

## Contributing
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D
