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
This area will be extended very soon especially for the skeleton, face and hdface streams.
* Color stream is saved in JPEG format with a csv containing a unix format timestamp and relative ticks information on all frames.
* Depth stream is saved in binary format with a csv containing a unix format timestamp and relative ticks information on all frames.
To reconstruct the image you can use 
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
* Body Index stream is saved in binary format with a csv containing a unix format timestamp and relative ticks information on all frames.
The above code works for the body index stream as well but you have to replace np.int16 with np.uint8.
* The skeleton is saved in csv format with comma separators. For each body frame all 6 bodies are saved detected or not.
* Face and FaceHD are saved in csv format as well with comma separator. Because face streams cannot be obtained with multisourceframe 
the rows contain the frames where the face detected is valid. All facial information is saved in the csvs (except for the face model parameters
in face hd though the vertices of the hd mesh are saved). 
* Audio is saved in .raw format while the beam information for each subframe is saved in .csv. To convert it to wav you can use sox:
```
$ sox -r 16000 -c 1 -b 32 -e floating_point audio.raw audio.wav
```
## Contributing
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D
