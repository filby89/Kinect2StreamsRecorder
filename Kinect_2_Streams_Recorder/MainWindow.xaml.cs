//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

using System;
using System.ComponentModel;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.IO;
using System.Windows;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using System.Threading;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters.Binary;
using System.Drawing;
using System.Drawing.Imaging;
using Ookii.Dialogs.Wpf;
using System.Windows.Media;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Threading;
using System.Windows.Media.Animation;

namespace Kinect_2_Streams_Recorder
{
    public unsafe struct ImageData
    {
        public byte[] pixels;
        public int counter;
        public TimeSpan relativeTime;
        public long timestamp;
        public UInt16 minDepth;
        public UInt16 maxDepth;
    }



    public unsafe struct AudioData
    {
        public byte[] pixels;
        public int counter;
        public TimeSpan relativeTime;
        public long timestamp;
        internal float beamAngle;
        internal float beamAngleConfidence;
        internal AudioBeamMode audioBeamMode;
        internal TimeSpan duration;
        internal uint frameLengthInBytes;
        internal ulong bodyTrackingId;
        internal TimeSpan relativeTimeStart;
    }

    public unsafe struct ImageDataUInt
    {
        public uint[] pixels;
        public int counter;
        public TimeSpan relativeTime;
        public long timestamp;
    }

    public unsafe struct BodyData
    {
        public Body[] bodies;
        public int counter;
        public TimeSpan relativeTime;
        public long timestamp;
    }

    public unsafe struct SingleFaceData
    {
        public FaceFrameResult face;
        public int counter;
        public TimeSpan relativeTime;
        public TimeSpan relativeTimeColor;
        public long timestamp;
        internal ulong trackingId;
        internal bool isTrackingIdValid;
    }

    public unsafe struct HDFaceData
    {
        public FaceAlignment hdface;
        public IReadOnlyList<CameraSpacePoint> vertices;
        public int counter;
        public TimeSpan relativeTime;
        public TimeSpan relativeTimeColor;
        public long timestamp;
        internal ulong trackingId;
        internal bool isTrackingIdValid;
    }

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// AUDIO STUFF ///
        /// <summary>
        /// Number of samples captured from Kinect audio stream each millisecond.
        /// </summary>
        private const int SamplesPerMillisecond = 16;

        /// <summary>
        /// Number of bytes in each Kinect audio stream sample (32-bit IEEE float).
        /// </summary>
        private const int BytesPerSample = sizeof(float);


        /// <summary>
        /// Will be allocated a buffer to hold a single sub frame of audio data read from audio stream.
        /// </summary>

        /// <summary>
        /// Buffer used to store audio stream energy data as we read audio.
        /// We store 25% more energy values than we strictly need for visualization to allow for a smoother
        /// stream animation effect, since rendering happens on a different schedule with respect to audio
        /// capture.
        /// </summary>


        /// <summary>
        /// Reader for audio frames
        /// </summary>
        private AudioBeamFrameReader audioReader = null;


        /// END AUDIO STUFF ///











        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;


        /// <summary>
        /// Array containing which streams to save each time
        /// </summary>
        private bool[] streamsToRecord = {false, false, false, false, false, false, false};

        /// <summary>
        /// Array containing recorded counters
        /// </summary>
        private int[] counters = { 0, 0, 0, 0, 0, 0, 0 };

        private int[] previousCounters = { 0, 0, 0, 0, 0, 0, 0 };

        /// <summary>
        /// Collection of colors to be used to display the BodyIndexFrame data.
        /// </summary>
        private static readonly uint[] BodyColor =
        {
            0x0000FF00,
            0x00FF0000,
            0xFFFF4000,
            0x40FFFF00,
            0xFF40FF00,
            0xFF808000,
        };

        /// <summary>
        /// Timer to count FPS
        /// </summary>
        private DispatcherTimer fpsTimer;

        /// <summary>
        /// Event interval for FPS timer
        /// </summary>
        private const int FpsInterval = 5;

        private Object colorLock = new Object();  

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for multi source frames
        /// </summary>
        private MultiSourceFrameReader multiSourceFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Description of the data contained in the color frame
        /// </summary>
        private FrameDescription colorFrameDescription = null;

        /// <summary>
        /// FaceModel is a result of capturing a face
        /// </summary>
        private FaceModel currentFaceModel = null;


        /// <summary>
        /// FaceModelBuilder is used to produce a FaceModel
        /// </summary>
        private FaceModelBuilder faceModelBuilder = null;


        TimeSpan first_frame_time;
        TimeSpan last_frame_time;

        private bool runConstantSecondThread = false;

        private bool stopMainColorSaveThread = false;
        private bool stopMainMultiSaveThread = false;

        /// <summary>
        /// Description of the data contained in the body index frame
        /// </summary>
        private FrameDescription bodyIndexFrameDescription = null;

        private string directoryToSave = null;
        private string colorDir = null;
        private string depthDir = null;
        private string bodyindexDir = null;
        private string faceDir = null;
        private string skelDir = null;
        private string hdfaceDir = null;
        private string audioDir = null;

        private System.IO.StreamWriter skelfile = null;
        private System.IO.StreamWriter facefile = null;
        private System.IO.StreamWriter hdfacefile = null;
        private System.IO.StreamWriter colorDataFile = null;
        private System.IO.StreamWriter depthDataFile = null;
        private System.IO.StreamWriter bodyIndexDataFile = null;
        private System.IO.StreamWriter audioDataFile = null;
        private string audioFile = null;
        private System.IO.StreamWriter calib = null;
        private System.IO.StreamWriter log = null;

        private int multiSourceFrameCounter = 0;

        private ConcurrentQueue<ImageData> colorBuffer = null;

        private ConcurrentQueue<ImageData> depthBuffer = null;

        private ConcurrentQueue<ImageData> bodyIndexBuffer = null;

        private ConcurrentQueue<BodyData> bodyBuffer = null;

        private ConcurrentQueue<SingleFaceData> faceBuffer = null;

        private ConcurrentQueue<HDFaceData> HDfaceBuffer = null;

        private ConcurrentQueue<AudioData> audioBuffer = null;

        private long timestamp;

        Thread colorSaveThread = null;
        Thread multiSaveThread = null;
        Thread colorSaveThread2 = null;
        Thread kinectThread = null;

        private int colorSaveThreadId;
        private int multiSaveThreadId;
        private bool stopSecondThreadInColor = false;
        private bool stopRecordingThreads = false;


        private DepthSpacePoint[] colorMappedToDepthPoints = null;
    

        private bool doubleSaveThread = false;

        /// <summary>
        /// Array to store bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// Number of bodies tracked
        /// </summary>
        private int bodyCount;

        /// <summary>
        /// Face frame sources
        /// </summary>
        private FaceFrameSource[] faceFrameSources = null;

        /// <summary>
        /// Face frame readers
        /// </summary>
        private FaceFrameReader[] faceFrameReaders = null;
        
        /// <summary>
        /// Face frame sources
        /// </summary>
        private HighDefinitionFaceFrameSource[] HDfaceFrameSources = null;

        /// <summary>
        /// Face frame readers
        /// </summary>
        private HighDefinitionFaceFrameReader[] HDfaceFrameReaders = null;

        /// <summary>
        /// Storage for face frame results
        /// </summary>
        private FaceFrameResult[] faceFrameResults = null;

        private bool startRecording = false;

        private bool wasRecording = false;

        private bool buffersNotEmpty = false;

        private string main_path_val = "E:\\";

        // number of kinect streams to consider
        private int streams_nr = 7;

        private int elapsedMultiSourceFramesCounter = 0;

        Stopwatch stopwatch_fps;
        
        Stopwatch stopwatch_session;

        AudioSource audioSource;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // open the reader for the multisourceframes frames
            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);

            this.colorBuffer = new ConcurrentQueue<ImageData>();

            this.depthBuffer = new ConcurrentQueue<ImageData>();

            this.bodyIndexBuffer = new ConcurrentQueue<ImageData>();

            this.bodyBuffer = new ConcurrentQueue<BodyData>();

            this.faceBuffer = new ConcurrentQueue<SingleFaceData>();

            this.HDfaceBuffer = new ConcurrentQueue<HDFaceData>();

            this.audioBuffer = new ConcurrentQueue<AudioData>();

            this.currentFaceModel = new FaceModel();

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            this.bodyIndexFrameDescription = this.kinectSensor.BodyIndexFrameSource.FrameDescription;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            this.colorMappedToDepthPoints = new DepthSpacePoint[this.colorFrameDescription.Width * this.colorFrameDescription.Height];

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the maximum number of bodies that would be tracked by Kinect
            this.bodyCount = this.kinectSensor.BodyFrameSource.BodyCount;

            // allocate storage to store body objects
            this.bodies = new Body[this.bodyCount];

            // specify the required face frame results
            FaceFrameFeatures faceFrameFeatures =
                FaceFrameFeatures.BoundingBoxInColorSpace
                | FaceFrameFeatures.BoundingBoxInInfraredSpace
                | FaceFrameFeatures.PointsInColorSpace
                | FaceFrameFeatures.PointsInInfraredSpace
                | FaceFrameFeatures.RotationOrientation
                | FaceFrameFeatures.FaceEngagement
                | FaceFrameFeatures.Glasses
                | FaceFrameFeatures.Happy
                | FaceFrameFeatures.LeftEyeClosed
                | FaceFrameFeatures.RightEyeClosed
                | FaceFrameFeatures.LookingAway
                | FaceFrameFeatures.MouthMoved
                | FaceFrameFeatures.MouthOpen;

            // create a face frame source + reader to track each face in the FOV
            this.faceFrameSources = new FaceFrameSource[this.bodyCount];
            this.faceFrameReaders = new FaceFrameReader[this.bodyCount];

            // Create an array of face sources/readers for each possible body.
            // These will be activated on demand as the corresponding bodies are tracked.
            this.HDfaceFrameSources = new HighDefinitionFaceFrameSource[this.bodyCount];
            this.HDfaceFrameReaders = new HighDefinitionFaceFrameReader[this.bodyCount];


            for (var i = 0; i < this.bodyCount; ++i)
            {
                 // Register as a handler for the face source data being returned by the Kinect.
                 this.HDfaceFrameSources[i] = new HighDefinitionFaceFrameSource(this.kinectSensor);
                 if (this.HDfaceFrameSources[i] == null)
                 {
                 }

                // Register as a handler for the face reader data being returned by the Kinect.
                this.HDfaceFrameReaders[i] = this.HDfaceFrameSources[i].OpenReader();
                 if (this.HDfaceFrameReaders[i] == null)
                 {
                 }
                 else
                 {
                     this.HDfaceFrameReaders[i].FrameArrived += this.onHDFaceFrameArrived;
                 }
             }


            for (int i = 0; i < this.bodyCount; i++)
            {
                // create the face frame source with the required face frame features and an initial tracking Id of 0
                this.faceFrameSources[i] = new FaceFrameSource(this.kinectSensor, 0, faceFrameFeatures);

                // open the corresponding reader
                this.faceFrameReaders[i] = this.faceFrameSources[i].OpenReader();
                if (this.faceFrameReaders[i] != null)
                {
                    // wire handler for face frame arrival
                    this.faceFrameReaders[i].FrameArrived += this.Reader_FaceFrameArrived;
                }
            }

            // Get its audio source
            audioSource = this.kinectSensor.AudioSource;

            // Allocate 1024 bytes to hold a single audio sub frame. Duration sub frame 
            // is 16 msec, the sample rate is 16khz, which means 256 samples per sub frame. 
            // With 4 bytes per sample, that gives us 1024 bytes.
            // this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];

            // Open the reader for the audio frames
            this.audioReader = audioSource.OpenReader();

            if (this.audioReader != null)
            {
                // Subscribe to new audio frame arrived events
                this.audioReader.FrameArrived += this.Audio_FrameArrived;
            }

            // Start fps timer
            this.fpsTimer = new DispatcherTimer(DispatcherPriority.SystemIdle);
            this.fpsTimer.Interval = new TimeSpan(0, 0, 0, 0, 500);
            this.fpsTimer.Tick += this.FpsTimerTick;
            this.fpsTimer.Start();

            this.stopwatch_fps = new Stopwatch();

            stopwatch_fps.Start();

            this.stopwatch_session = new Stopwatch();

            this.kinectThread = new Thread(new ThreadStart(this.main_event));
            this.kinectThread.Priority = ThreadPriority.Highest;
            this.kinectThread.IsBackground = true;
            this.kinectThread.Start();
            while (!this.kinectThread.IsAlive);

            this.open_faces();
            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        

        private void main_event() {
            // wire handler for frame arrival
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
        }




        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiSourceFrameReader != null)
            {
                this.multiSourceFrameReader.Dispose();
                this.multiSourceFrameReader = null;
            }

            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameReaders[i] != null)
                {
                    // FaceFrameReader is IDisposable
                    this.faceFrameReaders[i].Dispose();
                    this.faceFrameReaders[i] = null;
                }

                if (this.faceFrameSources[i] != null)
                {
                    // FaceFrameSource is IDisposable
                    this.faceFrameSources[i].Dispose();
                    this.faceFrameSources[i] = null;
                }

                if (this.HDfaceFrameReaders[i] != null)
                {
                    // FaceFrameReader is IDisposable
                    this.HDfaceFrameReaders[i].Dispose();
                    this.HDfaceFrameReaders[i] = null;
                }

            }

            if (this.audioReader != null)
            {
                // AudioBeamFrameReader is IDisposable
                this.audioReader.Dispose();
                this.audioReader = null;
            }

            // Stop timer
            if (null != this.fpsTimer)
            {
                this.fpsTimer.Stop();
                this.fpsTimer.Tick -= this.FpsTimerTick;
            }


            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            if (this.skelfile != null)
            {
                this.skelfile.Close();
                this.skelfile.Dispose();
                this.skelfile = null;
            }

            if (this.facefile != null)
            {
                this.facefile.Close();
                this.facefile.Dispose();
                this.facefile = null;
            }

            if (this.hdfacefile != null)
            {
                this.hdfacefile.Close();
                this.hdfacefile.Dispose();
                this.hdfacefile = null;
            }

            if (this.calib != null)
            {
                this.calib.Close();
                this.calib.Dispose();
                this.calib = null;
            }

            if (this.colorDataFile != null)
            {
                this.colorDataFile.Close();
                this.colorDataFile.Dispose();
                this.colorDataFile = null;
            }

            if (this.depthDataFile != null)
            {
                this.depthDataFile.Close();
                this.depthDataFile.Dispose();
                this.depthDataFile = null;
            }

            if (this.bodyIndexDataFile != null)
            {
                this.bodyIndexDataFile.Close();
                this.bodyIndexDataFile.Dispose();
                this.bodyIndexDataFile = null;
            }

            if (this.log != null)
            {
                this.log.Close();
                this.log.Dispose();
                this.log = null;
            }

        }


        /// <summary>
        /// Handles the audio frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Audio_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            if (this.streamsToRecord[6] && this.startRecording) {
                AudioBeamFrameReference frameReference = e.FrameReference;
                AudioBeamFrameList frameList = frameReference.AcquireBeamFrames();

                if (frameList != null)
                {
                    // AudioBeamFrameList is IDisposable
                    using (frameList)
                    {
                        // Only one audio beam is supported. Get the sub frame list for this beam
                        IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                        long audiotimestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();

                        // Loop over all sub frames, extract audio buffer and beam information
                        foreach (AudioBeamSubFrame subFrame in subFrameList)
                        {
                            // Process audio buffer

                            AudioData tempAudioData = new AudioData();

                            byte[] pixels = new byte[this.audioSource.SubFrameLengthInBytes];
                            
                            tempAudioData.pixels = pixels;

                            tempAudioData.relativeTimeStart = frameList[0].RelativeTimeStart;

                            tempAudioData.timestamp = audiotimestamp;
                            
                            tempAudioData.relativeTime = subFrame.RelativeTime;

                            tempAudioData.beamAngle = subFrame.BeamAngle;

                            tempAudioData.beamAngleConfidence = subFrame.BeamAngleConfidence;

                            tempAudioData.audioBeamMode = subFrame.AudioBeamMode;
                            
                            tempAudioData.duration = subFrame.Duration;

                            tempAudioData.frameLengthInBytes = subFrame.FrameLengthInBytes;

//                            IReadOnlyList<AudioBodyCorrelation> audioBodyCorrelation = subFrame.AudioBodyCorrelations;

                            foreach (AudioBodyCorrelation corr in subFrame.AudioBodyCorrelations)
                            {
                                tempAudioData.bodyTrackingId = corr.BodyTrackingId;
                            }

                            subFrame.CopyFrameDataToArray(pixels);

                            this.audioBuffer.Enqueue(tempAudioData);
                        }
                    }
                }                
            }
        }


        private void open_faces()
        {

            for (var i = 0; i < this.bodyCount; ++i)
            {
                // Register as a handler for the face source data being returned by the Kinect.
                this.HDfaceFrameSources[i] = new HighDefinitionFaceFrameSource(this.kinectSensor);
                if (this.HDfaceFrameSources[i] == null)
                {
                }

                // Register as a handler for the face reader data being returned by the Kinect.
                this.HDfaceFrameReaders[i] = this.HDfaceFrameSources[i].OpenReader();
                if (this.HDfaceFrameReaders[i] == null)
                {
                }
                else
                {
                    this.HDfaceFrameReaders[i].FrameArrived += this.onHDFaceFrameArrived;
                }
            }

        }


        /// <summary>
        /// Handles the multisource frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            if (multiSourceFrame != null)
            {
                this.multiSourceFrameCounter += 1;

                if (this.startRecording) {
                    this.elapsedMultiSourceFramesCounter += 1;
                    this.timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();
                }

                if (this.streamsToRecord[0] && this.startRecording)
                {

                    using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                    {
                        if (colorFrame != null)
                        {
                            FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                            using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                            {
                                ImageData tempImageData = new ImageData();
                                byte[] pixels = new byte[this.colorFrameDescription.Width * this.colorFrameDescription.Height *
                                    this.colorFrameDescription.BytesPerPixel];
                                tempImageData.pixels = pixels;
                                colorFrame.CopyConvertedFrameDataToArray(tempImageData.pixels, ColorImageFormat.Bgra);

                                tempImageData.counter = this.multiSourceFrameCounter;
                                tempImageData.relativeTime = colorFrame.RelativeTime;
                                tempImageData.timestamp = this.timestamp;

                                this.colorBuffer.Enqueue(tempImageData);
                                // this.log.WriteLine("Color Frame " + this.multiSourceFrameCounter + " added to buffer");
                            }
                        }
                    }
                }

                if (this.streamsToRecord[1] && this.startRecording)
                {
                    // push depth frame to buffer
                    using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                    {
                        if (depthFrame != null)
                        {
                            // the fastest way to process the body index data is to directly access 
                            // the underlying buffer
                            using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                            {
                                // Note: In order to see the full range of depth (including the less reliable far field depth)
                                // we are setting maxDepth to the extreme potential depth threshold
                                //ushort maxDepth = ushort.MaxValue;



                                // Allocate a new byte buffer to store this depth frame and timestamp.

                                ImageData tempImageData = new ImageData();

                                byte[] pixels = new byte[depthFrame.DepthFrameSource.FrameDescription.Height *
                                                     depthFrame.DepthFrameSource.FrameDescription.Width *
                                                     depthFrame.DepthFrameSource.FrameDescription.BytesPerPixel];

                                tempImageData.pixels = pixels;

                                tempImageData.counter = this.multiSourceFrameCounter;
                                tempImageData.relativeTime = depthFrame.RelativeTime;
                                tempImageData.timestamp = this.timestamp;
                                tempImageData.minDepth = depthFrame.DepthMinReliableDistance;
                                tempImageData.maxDepth = depthFrame.DepthMaxReliableDistance;
                                


                                // Convert the depth frame into the byte buffer.
                                using (var depthFrameBuffer = depthFrame.LockImageBuffer())
                                {
                                    Marshal.Copy(depthFrameBuffer.UnderlyingBuffer, tempImageData.pixels, 0, (int)depthFrameBuffer.Size);

                                    this.coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(
                                        depthFrameBuffer.UnderlyingBuffer,
                                        depthFrameBuffer.Size,
                                        this.colorMappedToDepthPoints);
                                }

                                this.depthBuffer.Enqueue(tempImageData);

                                // mapping
//                                ImageData tempImageDataMapped = new ImageData();
//                                tempImageDataMapped.pixels = this.colorMappedToDepthPoints;

//                                tempImageDataMapped.counter = this.multiSourceFrameCounter;
//                                tempImageDataMapped.relativeTime = depthFrame.RelativeTime;
//                                tempImageDataMapped.timestamp = this.timestamp;


  //                              this.colorMappedToDepthBuffer.Enqueue(tempImageDataMapped);
                                // end mapping

                                // If you wish to filter by reliable depth distance, uncomment the following line:
                                //// maxDepth = depthFrame.DepthMaxReliableDistance

                                //this.ProcessDepthFrameDataAndBuffer(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth, depthFrame.RelativeTime);
                            }
                        }
                    }
                }

                if (this.streamsToRecord[2] && this.startRecording)
                {
                    // push body index frame to buffer
                    using (BodyIndexFrame bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
                    {

                        if (bodyIndexFrame != null)
                        {

                            // the fastest way to process the body index data is to directly access 
                            // the underlying buffer
                            using (Microsoft.Kinect.KinectBuffer bodyIndexFrameBuffer = bodyIndexFrame.LockImageBuffer())
                            {


                                ImageData tempImageData = new ImageData();

                                byte[] pixels = new byte[bodyIndexFrame.BodyIndexFrameSource.FrameDescription.Height *
                                                     bodyIndexFrame.BodyIndexFrameSource.FrameDescription.Width *
                                                     bodyIndexFrame.BodyIndexFrameSource.FrameDescription.BytesPerPixel];
                                //Debug.WriteLine(bodyIndexFrame.BodyIndexFrameSource.FrameDescription.BytesPerPixel);
                                tempImageData.pixels = pixels;

                                tempImageData.counter = this.multiSourceFrameCounter;
                                tempImageData.relativeTime = bodyIndexFrame.RelativeTime;
                                tempImageData.timestamp = this.timestamp;

                                Marshal.Copy(bodyIndexFrameBuffer.UnderlyingBuffer, tempImageData.pixels, 0, (int)bodyIndexFrameBuffer.Size);

                                this.bodyIndexBuffer.Enqueue(tempImageData);
                            }
                        }
                    }
                }

                if ((this.streamsToRecord[3] || this.streamsToRecord[4] || this.streamsToRecord[5]) && this.startRecording)
                {
                    using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
                    {
                        if (bodyFrame != null)
                        {
                            BodyData bodyData = new BodyData();

                            var bodyArray = new Body[this.bodyCount];

                            bodyData.bodies = bodyArray;

                            bodyFrame.GetAndRefreshBodyData(bodyData.bodies);

                            bodyData.counter = this.multiSourceFrameCounter;
                            bodyData.relativeTime = bodyFrame.RelativeTime;
                            bodyData.timestamp = this.timestamp;

                            this.bodyBuffer.Enqueue(bodyData);
                            // this.log.WriteLine("Body Frame " + this.multiSourceFrameCounter + " added to buffer");

                            // faces now

                            // iterate through each face source
                            for (int i = 0; i < this.bodyCount; i++)
                            {
                                // check if a valid face is tracked in this face source
                                if (!this.faceFrameSources[i].IsTrackingIdValid)
                                {
                                    // check if the corresponding body is tracked 
                                    if (bodyData.bodies[i].IsTracked)
                                    {
                                        // update the face frame source to track this body
                                        this.faceFrameSources[i].TrackingId = bodyData.bodies[i].TrackingId;
                                        this.HDfaceFrameSources[i].TrackingId = bodyData.bodies[i].TrackingId;
                                    }
                                }
                                else
                                {

                                }
                            }
                        }
                    }
                }
            }
        }

        private void onHDFaceFrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            if (this.streamsToRecord[5] && this.startRecording)
            {
                // Retrieve face data for current frame.
                var frame = e.FrameReference.AcquireFrame();
                if (frame == null) return;

                long hdfacetimestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();

                using (frame)
                {
                    // Ignore untracked faces.
                    if (!frame.IsTrackingIdValid) return;
                    if (!frame.IsFaceTracked) return;


                    // Retrieve face alignment data.
                    var faceAlignment = new FaceAlignment();
                    frame.GetAndRefreshFaceAlignmentResult(faceAlignment);

                    HDFaceData tempHDFaceData = new HDFaceData();
                    tempHDFaceData.hdface = faceAlignment;
                    tempHDFaceData.vertices = this.currentFaceModel.CalculateVerticesForAlignment(faceAlignment);

                    tempHDFaceData.relativeTime = frame.BodyFrameReference.RelativeTime;
                    tempHDFaceData.relativeTimeColor = frame.ColorFrameReference.RelativeTime;

                    tempHDFaceData.timestamp = hdfacetimestamp;

                    tempHDFaceData.trackingId = frame.TrackingId;
                    tempHDFaceData.isTrackingIdValid = frame.IsTrackingIdValid;

                    this.HDfaceBuffer.Enqueue(tempHDFaceData);
                }
            }
        }



        private void setAndCreateDirectoriesAndFiles(string path)
        {
            this.directoryToSave = path;
            this.colorDir = Path.Combine(path, "Color");
            this.depthDir = Path.Combine(path, "Depth");
            this.bodyindexDir = Path.Combine(path, "BodyIndex");
            this.skelDir = Path.Combine(path, "Skeleton");
            this.faceDir = Path.Combine(path, "Face");
            this.hdfaceDir = Path.Combine(path, "HDFace");
            this.audioDir = Path.Combine(path, "Audio");

            System.IO.Directory.CreateDirectory(this.directoryToSave);

            if (this.streamsToRecord[0]) {
                System.IO.Directory.CreateDirectory(this.colorDir);
                this.colorDataFile = new System.IO.StreamWriter(Path.Combine(this.colorDir, "colorData.csv"));
            }

            if (this.streamsToRecord[1]) {
                System.IO.Directory.CreateDirectory(this.depthDir);
                this.depthDataFile = new System.IO.StreamWriter(Path.Combine(this.depthDir, "depthData.csv"));
            }

            if (this.streamsToRecord[2]) {
                System.IO.Directory.CreateDirectory(this.bodyindexDir);
                this.bodyIndexDataFile = new System.IO.StreamWriter(Path.Combine(this.bodyindexDir, "bodyIndexData.csv"));
            }

            if (this.streamsToRecord[3]) {
                System.IO.Directory.CreateDirectory(this.skelDir);
                this.skelfile = new System.IO.StreamWriter(Path.Combine(this.skelDir, "skeleton.csv"));
            }

            if (this.streamsToRecord[4]) {
                System.IO.Directory.CreateDirectory(this.faceDir);
                this.facefile = new System.IO.StreamWriter(Path.Combine(this.faceDir, "face.csv"));
            }

            if (this.streamsToRecord[5]) {
                System.IO.Directory.CreateDirectory(this.hdfaceDir);
                this.hdfacefile = new System.IO.StreamWriter(Path.Combine(this.hdfaceDir, "hdface.csv"));
            }

            if (this.streamsToRecord[6]) {
                System.IO.Directory.CreateDirectory(this.audioDir);
                this.audioDataFile = new System.IO.StreamWriter(Path.Combine(this.audioDir, "audioData.csv"));
                this.audioFile = Path.Combine(this.audioDir, "audio.raw");
            }

            this.calib = new System.IO.StreamWriter(Path.Combine(this.directoryToSave, "calib.txt"));
            this.log = new System.IO.StreamWriter(Path.Combine(this.directoryToSave, "log.txt"));

        }

        // /// <summary>
        // /// Directly accesses the underlying image buffer of the BodyIndexFrame to 
        // /// create a displayable bitmap.
        // /// This function requires the /unsafe compiler option as we make use of direct
        // /// access to the native memory pointed to by the bodyIndexFrameData pointer.
        // /// </summary>
        // /// <param name="bodyIndexFrameData">Pointer to the BodyIndexFrame image data</param>
        // /// <param name="bodyIndexFrameDataSize">Size of the BodyIndexFrame image data</param>
        // private unsafe void ProcessBodyIndexFrameDataAndBuffer(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize, TimeSpan relativeTime)
        // {
        //     byte* frameData = (byte*)bodyIndexFrameData;

        //     // allocate space to put the pixels being received and converted
        //     // uint[] bodyIndexPixels = new uint[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];

        //     ImageData tempImageData = new ImageData();

        //     // int[] bodyIndexData = new int[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];

        //     tempImageData.counter = this.multiSourceFrameCounter;
        //     tempImageData.relativeTime = relativeTime;
        //     tempImageData.timestamp = this.timestamp;



        //     for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
        //     {
        //         // the BodyColor array has been sized to match
        //         // BodyFrameSource.BodyCount
        //         if (frameData[i] < BodyColor.Length)
        //         {
        //             // this pixel is part of a player,
        //             // display the appropriate color
        //             bodyIndexData[i] = frameData[i];
        //             Debug.WriteLine(bodyIndexData[i]);
        //         }
        //         else
        //         {
        //             // this pixel is not part of a player
        //             // display black
        //             bodyIndexPixels[i] = 0;
        //         }
        //     }
        //     tempImageData.pixels = bodyIndexPixels;
        //     this.bodyIndexBuffer.Enqueue(tempImageData);
        //     // this.log.WriteLine("Body Index Frame " + this.multiSourceFrameCounter + " added to buffer");
        // }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameDataAndBuffer(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth, TimeSpan relativeTime)
        {
            // allocate space to put the pixels being received and converted
            byte[] depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            ImageData tempImageData = new ImageData();
            tempImageData.pixels = depthPixels;

            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                tempImageData.pixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }

            tempImageData.counter = this.multiSourceFrameCounter;
            tempImageData.relativeTime = relativeTime;
            tempImageData.timestamp = this.timestamp;

            this.depthBuffer.Enqueue(tempImageData);
        }


        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (null == this.kinectSensor)
            { 
                this.statusText.Text = string.Format("Kinect Sensor Not Available");
            }
            else
            {
                this.statusText.Text = string.Format("");
            }
        }


        /// <summary>
        /// Handles the face frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            if (this.streamsToRecord[4] && this.startRecording)
            {
                using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
                {
                    if (faceFrame != null)
                    {
                        long facetimestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();

                        // get the index of the face source from the face source array
                        int index = this.GetFaceSourceIndex(faceFrame.FaceFrameSource);

                        // check if this face frame has valid face frame results
                        if (this.ValidateFaceBoxAndPoints(faceFrame.FaceFrameResult))
                        {
                            // store this face frame result to draw later
                            //this.currentFaceFrameResults[index] = faceFrame.FaceFrameResult;

                            SingleFaceData facedata = new SingleFaceData();

                            facedata.face = faceFrame.FaceFrameResult;
                            facedata.relativeTime = faceFrame.BodyFrameReference.RelativeTime;
                            facedata.relativeTimeColor = faceFrame.ColorFrameReference.RelativeTime;
                            facedata.timestamp = facetimestamp;
                            facedata.trackingId = faceFrame.TrackingId;
                            facedata.isTrackingIdValid = faceFrame.IsTrackingIdValid;

                            this.faceBuffer.Enqueue(facedata);
                        }
                        else
                        {
                            // indicates that the latest face frame result from this reader is invalid
                            //this.currentFaceFrameResults[index] = null;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Validates face bounding box and face points to be within screen space
        /// </summary>
        /// <param name="faceResult">the face frame result containing face box and points</param>
        /// <returns>success or failure</returns>
        private bool ValidateFaceBoxAndPoints(FaceFrameResult faceResult)
        {
            bool isFaceValid = faceResult != null;

            if (isFaceValid)
            {
                var faceBox = faceResult.FaceBoundingBoxInColorSpace;
                if (faceBox != null)
                {
                    // check if we have a valid rectangle within the bounds of the screen space
                    isFaceValid = (faceBox.Right - faceBox.Left) > 0 &&
                                  (faceBox.Bottom - faceBox.Top) > 0 &&
                                  faceBox.Right <= this.colorFrameDescription.Width &&
                                  faceBox.Bottom <= this.colorFrameDescription.Height;

                    if (isFaceValid)
                    {
                        var facePoints = faceResult.FacePointsInColorSpace;
                        if (facePoints != null)
                        {
                            foreach (Microsoft.Kinect.PointF pointF in facePoints.Values)
                            {
                                // check if we have a valid face point within the bounds of the screen space
                                bool isFacePointValid = pointF.X > 0.0f &&
                                                        pointF.Y > 0.0f &&
                                                        pointF.X < this.colorFrameDescription.Width &&
                                                        pointF.Y < this.colorFrameDescription.Height;

                                if (!isFacePointValid)
                                {
                                    isFaceValid = false;
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            return isFaceValid;
        }


        /// <summary>
        /// Returns the index of the face frame source
        /// </summary>
        /// <param name="faceFrameSource">the face frame source</param>
        /// <returns>the index of the face source in the face source array</returns>
        private int GetFaceSourceIndex(FaceFrameSource faceFrameSource)
        {
            int index = -1;

            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameSources[i] == faceFrameSource)
                {
                    index = i;
                    break;
                }
            }

            return index;
        }


        private void saveColor()
        {
            while(true) {            
                ImageData tempImageData;
                lock (colorLock)
                {
                    if (this.colorBuffer.Count > 0) { 
                        this.colorBuffer.TryDequeue(out tempImageData);
                    }
                    else {
                        // I put this here so that it will not run all the time
                        if (this.stopMainColorSaveThread == true) {
                            this.stopMainColorSaveThread = false;
                            return;
                        }

                        continue;
                    }
                }

                byte []pixels = tempImageData.pixels;
                // Allocate a new byte buffer to store this RGB frame and timestamp.

                string path = Path.Combine(this.colorDir, tempImageData.relativeTime.Ticks + "_" + tempImageData.counter + ".jpg");

                var colorRect = new Rectangle(0, 0,
                                              1920,
                                              1080);

                // Wrap RGB frames into bitmap buffers.
                var bmp32 = new Bitmap(1920,
                                       1080,
                                       System.Drawing.Imaging.PixelFormat.Format32bppRgb);


                // Lock the bitmap's bits.
                System.Drawing.Imaging.BitmapData bmpData =
                    bmp32.LockBits(colorRect, System.Drawing.Imaging.ImageLockMode.ReadWrite, bmp32.PixelFormat);
                IntPtr bmpPtr = bmpData.Scan0;
                Marshal.Copy(pixels, 0, bmpPtr, pixels.Length);
                bmp32.UnlockBits(bmpData);

                // bmp32.Save(path, ImageFormat.Png);

                ImageCodecInfo jpgEncoder = GetEncoder(ImageFormat.Jpeg);

                // Create an Encoder object based on the GUID
                // for the Quality parameter category.
                System.Drawing.Imaging.Encoder myEncoder =
                    System.Drawing.Imaging.Encoder.Quality;

                // Create an EncoderParameters object.
                // An EncoderParameters object has an array of EncoderParameter
                // objects. In this case, there is only one
                // EncoderParameter object in the array.
                EncoderParameters myEncoderParameters = new EncoderParameters(1);

                EncoderParameter myEncoderParameter = new EncoderParameter(myEncoder, 100L);
                myEncoderParameters.Param[0] = myEncoderParameter;
                
                bmp32.Save(path, jpgEncoder, myEncoderParameters);
                lock (this.colorDataFile)
                {
                    this.colorDataFile.WriteLine(tempImageData.counter + ";" + tempImageData.relativeTime.Ticks + ";" + tempImageData.timestamp);
                }
                bmp32.Dispose();
                bmp32 = null;

                if (this.counters[0] == 0) {
                    this.first_frame_time = tempImageData.relativeTime;
                }
                else {
                    this.last_frame_time = tempImageData.relativeTime;
                }
                this.counters[0] += 1;
                if (runConstantSecondThread == false) {
                    if (this.stopSecondThreadInColor == true){
                        Thread thread = Thread.CurrentThread;
                        if (thread.ManagedThreadId  != colorSaveThreadId) {
                            this.doubleSaveThread = false;
                            this.stopSecondThreadInColor = false;
                            lock (this.log)
                            {
                                this.log.WriteLine("Stopping the second thread to save color stream");
                            }
                            return;
                        }
                    }
                }
                // this.elapsedRecordedFrames[0] += 1;   
            }
        }

        private void saveMulti()
        {
            while (true)
            {
                this.saveDepth();
                this.saveBodyIndex();
                this.saveBody();
                this.saveFace();
                this.saveHDFace();
                this.saveAudio();
                if (this.stopMainMultiSaveThread == true) {
                    this.stopMainMultiSaveThread = false;
                    return;
                }
            }
        }

        private void saveDepth()
        {
            if (this.depthBuffer.Count != 0) {
    
                ImageData tempImageData;

                this.depthBuffer.TryDequeue(out tempImageData);

                byte []pixels = tempImageData.pixels;
                
                string path = Path.Combine(this.depthDir, tempImageData.relativeTime.Ticks + "_" + tempImageData.counter + ".txt");

                File.WriteAllBytes(path, pixels);

                this.depthDataFile.WriteLine(tempImageData.counter + ";" + tempImageData.relativeTime.Ticks + ";" + ";" + tempImageData.timestamp + ";" + tempImageData.minDepth + ";" + tempImageData.maxDepth);

                pixels = null;
                this.counters[1] += 1;
            }
        }

        private void saveBodyIndex()
        {
            if (this.bodyIndexBuffer.Count != 0)
            {

                ImageData tempImageData;

                this.bodyIndexBuffer.TryDequeue(out tempImageData);

                string path = Path.Combine(this.bodyindexDir, tempImageData.relativeTime.Ticks + "_" + tempImageData.counter + ".txt");

                byte[] pixels = tempImageData.pixels;

                File.WriteAllBytes(path, pixels);

                this.bodyIndexDataFile.WriteLine(tempImageData.counter + ";" + tempImageData.relativeTime.Ticks + ";" + tempImageData.timestamp);

                //                 var bodyIndexRect = new Rectangle(0, 0,
                //                                               this.bodyIndexFrameDescription.Width,
                //                                               this.bodyIndexFrameDescription.Height);

                //                 // Wrap RGB frames into bitmap buffers.
                //                 var bmp32 = new Bitmap(this.bodyIndexFrameDescription.Width,
                //                                        this.bodyIndexFrameDescription.Height,
                //                                        System.Drawing.Imaging.PixelFormat.Format32bppRgb);

                //                 string path = Path.Combine(this.bodyindexDir, tempImageData.relativeTime.Ticks + "_" + tempImageData.counter + ".bmp");



                //                 // Lock the bitmap's bits.
                //                 System.Drawing.Imaging.BitmapData bmpData =
                //                                   bmp32.LockBits(bodyIndexRect, System.Drawing.Imaging.ImageLockMode.ReadWrite, bmp32.PixelFormat);
                //                             IntPtr bmpPtr = bmpData.Scan0;
                // //                          Marshal.Copy(pixels, 0, bmpPtr, pixels.Length);
                //                   bmp32.UnlockBits(bmpData);

                //                  bmp32.Save(path, ImageFormat.Bmp);

                //                  this.bodyIndexDataFile.WriteLine(tempImageData.counter + "," + tempImageData.relativeTime.Ticks + "," + tempImageData.timestamp);

                pixels = null;
                this.counters[2] += 1;
                // this.elapsedRecordedFrames[2] += 1;
            }
        }


        private void saveBody()
        {
            if (this.bodyBuffer.Count != 0)
            {

                BodyData tempBodyData;

                this.bodyBuffer.TryDequeue(out tempBodyData);

                Body[] bodies = tempBodyData.bodies;

                foreach (Body body in bodies)
                {
                    this.skelfile.Write(tempBodyData.counter + ";" + tempBodyData.relativeTime.Ticks + ";" + tempBodyData.timestamp + ";");

                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                    // convert the joint points to depth (display) space
                    Dictionary<JointType, System.Windows.Point> jointPoints = new Dictionary<JointType, System.Windows.Point>();

                    IReadOnlyDictionary<JointType, JointOrientation> JointOrientations = body.JointOrientations;


                    foreach (JointType jointType in joints.Keys)
                    {
                        // sometimes the depth(Z) of an inferred joint may show as negative
                        // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                        CameraSpacePoint position = joints[jointType].Position;
                        if (position.Z < 0)
                        {
                            position.Z = InferredZPositionClamp;
                        }

                        DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);

                        ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);

                        this.skelfile.Write(jointType + ";" + body.Joints[jointType].TrackingState + ";" + body.Joints[jointType].Position.X + ";" + body.Joints[jointType].Position.Y + ";" + body.Joints[jointType].Position.Z + ";" +
                            depthSpacePoint.X + ";" + depthSpacePoint.Y + ";" + colorSpacePoint.X + ";" + colorSpacePoint.Y + ";" + JointOrientations[jointType].Orientation.X + ";" + JointOrientations[jointType].Orientation.Y + ";" +
                            JointOrientations[jointType].Orientation.Z + ";" + JointOrientations[jointType].Orientation.W + ";");
                    }

                    this.skelfile.Write(body.ClippedEdges + ";" + body.HandLeftState + ";" + body.HandLeftConfidence +
                            ";" + body.HandRightState + ";" + body.HandRightConfidence + ";" +
                            body.IsRestricted + ";" + body.IsTracked + ";" + Body.JointCount + ";" +
                            body.TrackingId + ";" + body.Lean.X + ";" + body.Lean.Y + ";" +
                            body.LeanTrackingState + "\n");

                }
                this.counters[3] += 1;
                // this.elapsedRecordedFrames[3] += 1;
            }
        }


        private ImageCodecInfo GetEncoder(ImageFormat format)
        {
            ImageCodecInfo[] codecs = ImageCodecInfo.GetImageDecoders();
            foreach (ImageCodecInfo codec in codecs)
            {
                if (codec.FormatID == format.Guid)
                {
                    return codec;
                }
            }
            return null;
        }

        private void saveAudio()
        {
            if (this.audioBuffer.Count != 0)
            {

                AudioData tempAudioData;

                this.audioBuffer.TryDequeue(out tempAudioData);

                this.audioDataFile.Write(tempAudioData.relativeTimeStart.Ticks + ";" + tempAudioData.relativeTime.Ticks + ";" + tempAudioData.timestamp + ";" + 
                    tempAudioData.beamAngle + ";" + tempAudioData.beamAngleConfidence + ";" + 
                    tempAudioData.audioBeamMode + ";" + tempAudioData.duration + ";" + 
                    tempAudioData.frameLengthInBytes + ";" + tempAudioData.bodyTrackingId + "\n");

                AppendAllBytes(this.audioFile, tempAudioData.pixels);
                
                tempAudioData.pixels = null;

                counters[6] += 1;
            }
        }

        public static void AppendAllBytes(string path, byte[] bytes)
        {
            //argument-checking here.

            using (var stream = new FileStream(path, FileMode.Append))
            {
                stream.Write(bytes, 0, bytes.Length);
            }
        }

        private void saveFace()
        {
            if (this.faceBuffer.Count != 0)
            {


                SingleFaceData tempFaceData;

                this.faceBuffer.TryDequeue(out tempFaceData);

                FaceFrameResult face = tempFaceData.face;

                if (face != null)
                {
                    this.facefile.Write(tempFaceData.relativeTime.Ticks + ";" + tempFaceData.relativeTimeColor.Ticks + ";" + tempFaceData.timestamp + ";" + face.TrackingId + ";" + tempFaceData.isTrackingIdValid + ";");
//                    this.hdfacefile.Write(tempHDFaceData.relativeTime.Ticks + ";" + tempHDFaceData.relativeTimeColor.Ticks + ";" + tempHDFaceData.timestamp + ";" + tempHDFaceData.trackingId + ";" + tempHDFaceData.isTrackingIdValid + ";");

                    this.facefile.Write(face.FaceBoundingBoxInColorSpace.Bottom + ";" + face.FaceBoundingBoxInColorSpace.Left + ";" + face.FaceBoundingBoxInColorSpace.Top + ";" + face.FaceBoundingBoxInColorSpace.Right +
                        ";" + face.FaceBoundingBoxInInfraredSpace.Bottom + ";" + face.FaceBoundingBoxInInfraredSpace.Left + ";" + face.FaceBoundingBoxInInfraredSpace.Top + ";" + face.FaceBoundingBoxInInfraredSpace.Right +
                        ";");

                    IReadOnlyDictionary<FaceProperty, DetectionResult> faceproperties = face.FaceProperties;

                    foreach (FaceProperty faceProperty in faceproperties.Keys)
                    {
                        this.facefile.Write(faceProperty + ";" + face.FaceProperties[faceProperty] + ";");
                    }

                    IReadOnlyDictionary<FacePointType, Microsoft.Kinect.PointF> facepoints = face.FacePointsInColorSpace;

                    foreach (FacePointType facePointType in facepoints.Keys)
                    {
                        this.facefile.Write(facePointType + ";" + face.FacePointsInColorSpace[facePointType].X + ";" + face.FacePointsInColorSpace[facePointType].Y + ";");
                    }

                    facepoints = face.FacePointsInInfraredSpace;

                    foreach (FacePointType facePointType in facepoints.Keys)
                    {
                        this.facefile.Write(facePointType + ";" + face.FacePointsInInfraredSpace[facePointType].X + ";" + face.FacePointsInInfraredSpace[facePointType].Y + ";");
                    }

                    this.facefile.Write(face.FaceRotationQuaternion.X + ";" + face.FaceRotationQuaternion.Y + ";" +
                        face.FaceRotationQuaternion.Z + ";" + face.FaceRotationQuaternion.W + ";");

                    double pitch, roll, yaw = 0;

                    ExtractFaceRotationInDegrees(
                        face.FaceRotationQuaternion,
                            out pitch, out yaw, out roll);

                    this.facefile.Write(yaw + ";" + pitch + ";" + roll + "\n");

                    this.counters[4] += 1;
                }

            }
        }

        private static void ExtractFaceRotationInDegrees(Vector4 rotQuaternion, out double pitch, out double yaw, out double roll)
        {
            double x = rotQuaternion.X;
            double y = rotQuaternion.Y;
            double z = rotQuaternion.Z;
            double w = rotQuaternion.W;

            // convert face rotation quaternion to Euler angles in degrees
            pitch = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) -
                (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
            yaw = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
            roll = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) +
                (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;
        }

        public static T DeepCopy<T>(T other)
        {
            using (MemoryStream ms = new MemoryStream())
            {
                BinaryFormatter formatter = new BinaryFormatter();
                formatter.Serialize(ms, other);
                ms.Position = 0;
                return (T)formatter.Deserialize(ms);
            }
        }

        private void saveHDFace()
        {
            if (this.HDfaceBuffer.Count != 0)
            {


                HDFaceData tempHDFaceData;

                this.HDfaceBuffer.TryDequeue(out tempHDFaceData);

                FaceAlignment hdface = tempHDFaceData.hdface;

                var vertices = tempHDFaceData.vertices;

                this.hdfacefile.Write(tempHDFaceData.relativeTime.Ticks + ";" + tempHDFaceData.relativeTimeColor.Ticks + ";" + tempHDFaceData.timestamp + ";" + tempHDFaceData.trackingId + ";" + tempHDFaceData.isTrackingIdValid + ";");

                DepthSpacePoint depthSpacePoint;
                ColorSpacePoint colorSpacePoint;

                for (int i = 0; i < vertices.Count; i++)
                {
                    // edw to hdfacefile kolaei !
                    this.hdfacefile.Write(vertices[i].X + ";" + vertices[i].Y + ";" + vertices[i].Z + ";");

                    depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(vertices[i]);

                    colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(vertices[i]);
/*
                    Debug.WriteLine(vertices[i].X);
                    Debug.WriteLine(vertices[i].Y);
                    Debug.WriteLine(vertices[i].Z);
                    */
                    this.hdfacefile.Write(depthSpacePoint.X + ";" + depthSpacePoint.Y + ";");
                    this.hdfacefile.Write(colorSpacePoint.X + ";" + colorSpacePoint.Y + ";");
                }


                IReadOnlyDictionary<FaceShapeAnimations, float> faceAnimations = hdface.AnimationUnits;


                foreach (FaceShapeAnimations faceShapeAnimation in faceAnimations.Keys)
                {
                    this.hdfacefile.Write(faceShapeAnimation + ";" + hdface.AnimationUnits[faceShapeAnimation] + ";");
                }

                this.hdfacefile.Write(hdface.FaceOrientation.X + ";" + hdface.FaceOrientation.Y + ";" +
                    hdface.FaceOrientation.Z + ";" + hdface.FaceOrientation.W + ";");

                double pitch, roll, yaw = 0;

                ExtractFaceRotationInDegrees(
                    hdface.FaceOrientation,
                        out pitch, out yaw, out roll);

                this.hdfacefile.Write(yaw + ";" + pitch + ";" + roll + ";");

                this.hdfacefile.Write(hdface.HeadPivotPoint.X + ";" + hdface.HeadPivotPoint.Y + ";" + hdface.HeadPivotPoint.Z + ";");

                depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(hdface.HeadPivotPoint);

                colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(hdface.HeadPivotPoint);

                this.hdfacefile.Write(depthSpacePoint.X + ";" + depthSpacePoint.Y + ";");

                this.hdfacefile.Write(colorSpacePoint.X + ";" + colorSpacePoint.Y + ";");

                this.hdfacefile.Write(hdface.Quality + "\n");

                this.counters[5] += 1;
                // this.elapsedRecordedFrames[5] += 1;
            }
        }



        private void GetTables()
        {
            Microsoft.Kinect.PointF[] depthToCameraTable = this.coordinateMapper.GetDepthFrameToCameraSpaceTable();
            int len = 0;
            for (int i = 0; i < depthToCameraTable.GetLength(len); i++)
            {
                this.calib.Write(depthToCameraTable[i].X + ";" + depthToCameraTable[i].Y);
            }

            var intrinsics = this.coordinateMapper.GetDepthCameraIntrinsics();
            this.calib.Write(intrinsics.FocalLengthX + ";" + intrinsics.FocalLengthY + ";" +
                intrinsics.PrincipalPointX + ";" + intrinsics.PrincipalPointY + ";" +
                intrinsics.RadialDistortionSecondOrder + ";" + intrinsics.RadialDistortionFourthOrder + ";" +
                intrinsics.RadialDistortionSixthOrder);

            this.calib.Close();
        }


        /// <summary>
        /// Checks that everything is ok and updates the fps
        /// </summary>
        /// <param name="sender">Object sending the event</param>
        /// <param name="e">Event arguments</param>
        private void FpsTimerTick(object sender, EventArgs e)
        {
           if (this.kinectSensor == null)
            {
                this.statusText.Text = "Kinect Sensor Not Available";
            }
            else {
                this.stopwatch_fps.Stop();

                // Calculate time span from last calculation of FPS
                this.statusText.Text = ((double)this.elapsedMultiSourceFramesCounter / this.stopwatch_fps.Elapsed.TotalSeconds).ToString();

                this.elapsedMultiSourceFramesCounter = 0;

                this.faceFramesInBuffer.Content = this.faceBuffer.Count;
                this.colorFramesInBuffer.Content = this.colorBuffer.Count;
                this.depthFramesInBuffer.Content = this.depthBuffer.Count;
                this.bodyIndexFramesInBuffer.Content = this.bodyIndexBuffer.Count;
                this.HDFaceFramesInBuffer.Content = this.HDfaceBuffer.Count;
                this.skeletonFramesInBuffer.Content = this.bodyBuffer.Count;
                this.audioFramesInBuffer.Content = this.audioBuffer.Count;

                if (runConstantSecondThread == false)
                {
                    // if we have many color frames allocate a second thread to them
                    if (this.colorBuffer.Count > 20 && this.doubleSaveThread == false) {
                        lock (this.log)
                        {
                            this.log.WriteLine("Allocating a second thread to save color stream");
                        }
                        this.doubleSaveThread = true;
                        this.colorSaveThread2 = new Thread(new ThreadStart(this.saveColor));
                        this.colorSaveThread2.IsBackground = true;
                        this.colorSaveThread2.Start();
                        while (!this.colorSaveThread2.IsAlive);                
                    }

                    // release the thread if all ok
                    if (this.colorBuffer.Count < 5 && this.doubleSaveThread == true) {
                        this.stopSecondThreadInColor = true;
                    }
                }

                // Spin for a while waiting for the started thread to become
                // alive:


                if (this.faceBuffer.Count + this.colorBuffer.Count + this.depthBuffer.Count + this.bodyIndexBuffer.Count + this.HDfaceBuffer.Count + this.bodyBuffer.Count + this.audioBuffer.Count > 0) {
                    this.buffersNotEmpty = true;
                }
                else {
                    this.buffersNotEmpty = false;
                    if (this.wasRecording) {

                        this.statusText.Text = "Recording session finished!";

                        long ticks = this.last_frame_time.Ticks - this.first_frame_time.Ticks;

                        long ideal_frames = ticks / 10000000 * 30;

                        double ideal_frames_audio = ticks / 10000000.0 * 1000/16;
                        lock (this.log)
                        {
                            this.log.WriteLine("Color frames recorded: " + this.counters[0]);
                            this.log.WriteLine("Depth frames recorded: " + this.counters[1]);
                            this.log.WriteLine("Body Index frames recorded: " + this.counters[2]);
                            this.log.WriteLine("Skeleton frames recorded: " + this.counters[3]);
                            this.log.WriteLine("Face frames recorded: " + this.counters[4]);
                            this.log.WriteLine("HDFace frames recorded: " + this.counters[5]);
                            this.log.WriteLine("Audio frames recorded: " + this.counters[6]);
                            this.log.WriteLine("Seconds recorded: " + ticks / 10000000);
                            this.log.WriteLine("Ticks recorded: " + ticks + 
                                " Frames Number should be: " + ideal_frames + 
                                " Audio Frames Number should be: " + ideal_frames_audio);
                                // this.log.WriteLine("According with kinect relative times, " + 
                                //     " frames Number should be: " + ideal_frames_by_kinect_relative_times);

                        }

                        this.browse_button.IsEnabled = true;
                        this.secondary_path.IsEnabled = true;
                        this.colorCheckbox.IsEnabled = true;
                        this.depthCheckbox.IsEnabled = true;
                        this.bodyIndexCheckbox.IsEnabled = true;
                        this.audioCheckbox.IsEnabled = true;
                        this.skeletonCheckbox.IsEnabled = true;
                        this.faceCheckbox.IsEnabled = true;
                        this.HDFaceCheckbox.IsEnabled = true;
                        this.constantSecondThreadColor.IsEnabled = true;

                        if (this.skelfile != null)
                        {
                            this.skelfile.Close();
                            this.skelfile.Dispose();
                            this.skelfile = null;
                        }

                        if (this.facefile != null)
                        {
                            this.facefile.Close();
                            this.facefile.Dispose();
                            this.facefile = null;
                        }

                        if (this.hdfacefile != null)
                        {
                            this.hdfacefile.Close();
                            this.hdfacefile.Dispose();
                            this.hdfacefile = null;
                        }

                        if (this.calib != null)
                        {
                            this.calib.Close();
                            this.calib.Dispose();
                            this.calib = null;
                        }

                        if (this.log != null)
                        {
                            this.log.Close();
                            this.log.Dispose();
                            this.log = null;
                        }

                        if (this.colorDataFile != null)
                        {
                            this.colorDataFile.Close();
                            this.colorDataFile.Dispose();
                            this.colorDataFile = null;
                        }

                        if (this.depthDataFile != null)
                        {
                            this.depthDataFile.Close();
                            this.depthDataFile.Dispose();
                            this.depthDataFile = null;
                        }

                        if (this.bodyIndexDataFile != null)
                        {
                            this.bodyIndexDataFile.Close();
                            this.bodyIndexDataFile.Dispose();
                            this.bodyIndexDataFile = null;
                        }


                        if (this.audioDataFile != null)
                        {
                            this.audioDataFile.Close();
                            this.audioDataFile.Dispose();
                            this.audioDataFile = null;
                        }

                        this.wasRecording = false;

                        this.stopMainMultiSaveThread = true;
                        this.stopMainColorSaveThread = true;
                        start.Content = "Start Recording";
                    }
                }

                this.stopwatch_fps.Restart();
            }
        }

        // /// <summary>
        // /// Handler for FPS timer tick
        // /// </summary>
        // /// <param name="sender">Object sending the event</param>
        // /// <param name="e">Event arguments</param>
        // private void RpsTimerTick(object sender, EventArgs e)
        // {
        //     if (this.startRecording) {
        //         this.stopwatch_recording.Stop();

        //         this.recordFpsText.Text = ((double)this.counters / this.stopwatch_recording.Elapsed.TotalSeconds).ToString();

        //         // Reset frame counter
        //         this.elapsedMultiSourceFramesCounter = 0;
        //         this.stopwatch_recording.Restart();
        //         // this.lastFPSTimestamp = DateTime.UtcNow;
        //     }
        // }

        private void StreamCheckbox_Checked(object sender, EventArgs e)
        {
            int idx = Convert.ToInt32(((CheckBox)sender).Tag);
            this.streamsToRecord[idx] = ((CheckBox)sender).IsChecked.Value;
            checkAllCheckboxesValues();
        }

        private void StreamCheckbox_Unchecked(object sender, EventArgs e)
        {
            int idx = Convert.ToInt32(((CheckBox)sender).Tag);
            this.streamsToRecord[idx] = ((CheckBox)sender).IsChecked.Value;
            checkAllCheckboxesValues();
        }

        private void SelectAll_Checked(object sender, RoutedEventArgs e)
        {
            for (int i=0; i<streams_nr; i++)
            {
                this.streamsToRecord[i] = true;
            }
            colorCheckbox.IsChecked = true;
            depthCheckbox.IsChecked = true;
            skeletonCheckbox.IsChecked = true;
            bodyIndexCheckbox.IsChecked = true;
            HDFaceCheckbox.IsChecked = true;
            faceCheckbox.IsChecked = true;
            audioCheckbox.IsChecked = true;
        }

        private void SelectAll_Unchecked(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < streams_nr; i++)
            {
                this.streamsToRecord[i] = false;
            }
            colorCheckbox.IsChecked = false;
            depthCheckbox.IsChecked = false;
            skeletonCheckbox.IsChecked = false;
            bodyIndexCheckbox.IsChecked = false;
            HDFaceCheckbox.IsChecked = false;
            faceCheckbox.IsChecked = false;
            audioCheckbox.IsChecked = false;
            this.start.IsEnabled = false;
        }


        private void checkAllCheckboxesValues()
        {
            for (int i=0; i<streams_nr; i++)
            {
                if (this.streamsToRecord[i] == true && this.kinectSensor != null)
                {
                    this.start.IsEnabled = true;
                    return;
                }
                else
                {
                    this.start.IsEnabled = false;
                }
            }
        }

        private void start_Click(object sender, RoutedEventArgs e)
        {
            if (!this.startRecording) {
                // get the directory
                var secondary_path_val = secondary_path.Text;

                //                this.directoryToSave = Path.Combine("C:\\Kinect2StreamsRecorder", DateTimeOffset.Now.ToString("yyyy_MM_dd-HH_mm_ss"));
                if (!string.IsNullOrWhiteSpace(secondary_path_val)) {
                    this.directoryToSave = Path.Combine(main_path_val, secondary_path_val , DateTimeOffset.Now.ToString("yyyy_MM_dd-HH_mm_ss"));
                }
                else
                {
                    this.directoryToSave = Path.Combine(main_path_val, "Kinect2StreamsRecorder", DateTimeOffset.Now.ToString("yyyy_MM_dd-HH_mm_ss"));
                }

                this.browse_button.IsEnabled = false;
                this.secondary_path.IsEnabled = false;
                this.colorCheckbox.IsEnabled = false;
                this.depthCheckbox.IsEnabled = false;
                this.bodyIndexCheckbox.IsEnabled = false;
                this.skeletonCheckbox.IsEnabled = false;
                this.faceCheckbox.IsEnabled = false;
                this.HDFaceCheckbox.IsEnabled = false;
                this.audioCheckbox.IsEnabled = false;
                this.constantSecondThreadColor.IsEnabled = false;

                setAndCreateDirectoriesAndFiles(this.directoryToSave);
                // reset all counters
                for (int i=0;i<6;i++) {
                    this.counters[i] = 0;
                    this.previousCounters[i] = 0;
                }

                this.colorSaveThread = new Thread(new ThreadStart(this.saveColor));
                this.colorSaveThread.IsBackground = true;
                this.colorSaveThread.Start();

                this.multiSaveThread = new Thread(new ThreadStart(this.saveMulti));
                this.multiSaveThread.IsBackground = true;
                this.multiSaveThread.Start();

                while (!this.colorSaveThread.IsAlive);

                while (!this.multiSaveThread.IsAlive);


                this.colorSaveThreadId = colorSaveThread.ManagedThreadId;
                this.multiSaveThreadId = multiSaveThread.ManagedThreadId;


                if (constantSecondThreadColor.IsChecked == true) {
                    this.colorSaveThread2 = new Thread(new ThreadStart(this.saveColor));
                    this.colorSaveThread2.IsBackground = true;
                    this.colorSaveThread2.Start();
                    this.runConstantSecondThread = true; 
                }

                // start.Background = new SolidColorBrush(System.Windows.Media.Color.FromArgb(0, 244, 67, 54));
                start.Content = "Stop Recording";
                this.GetTables();

                this.startRecording = true;
                // this.stopwatch_session.Restart();
            }
            else
            {
                this.startRecording = false;
                // start.Background = new SolidColorBrush(System.Windows.Media.Color.FromArgb(0, 33, 150, 243));

                this.wasRecording = true;

                // this.stopwatch_session.Stop();

                this.statusText.Text = "Please wait while buffered frames are being saved!";
            }
        }

        private void browse_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new Ookii.Dialogs.Wpf.VistaFolderBrowserDialog();
            if (dialog.ShowDialog(this).GetValueOrDefault())
            {
                textBoxFolderPath.Text = dialog.SelectedPath;
                this.main_path_val = dialog.SelectedPath;
            }
        }


    }
}


