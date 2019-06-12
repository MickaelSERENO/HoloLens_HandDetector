using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Media.MediaProperties;
using HandDetector_Native;
using Windows.Media;
using System.Linq;
using System.Numerics;
using Windows.Perception.Spatial;

namespace Sereno.HandDetector
{
    /// <summary>
    /// Hand Detector class. Permits to detect hands via a depth camera
    /// </summary>
    public class HandDetector
    {
        private MediaFrameSourceGroup m_mediaGroup   = null;
        private MediaFrameSourceInfo  m_mediaInfo    = null;
        private MediaCapture          m_mediaCapture = null;
        private HDMediaSinkProxy      m_mediaSink    = null;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="group"></param>
        /// <param name="info"></param>
        private HandDetector(MediaFrameSourceGroup group, MediaFrameSourceInfo info)
        {
            m_mediaGroup = group;
            m_mediaInfo  = info;
            m_mediaSink = null;
        }

        /// <summary>
        /// List on screen the available media frame source information
        /// </summary>
        public static async void ListMediaFrameSourceInfo()
        {
            IReadOnlyList<MediaFrameSourceGroup> allFrameSourceGroups = await MediaFrameSourceGroup.FindAllAsync();

            //Print all devices
            foreach (MediaFrameSourceGroup group in allFrameSourceGroups)
            {
                Debug.WriteLine("-------------------------------------------------------------");
                Debug.WriteLine($"Group: {group.DisplayName}");
                foreach (MediaFrameSourceInfo info in group.SourceInfos)
                {
                    Debug.WriteLine($"SourceKind : {info.SourceKind}");
                    Debug.WriteLine($"Device : {info.DeviceInformation.Name}:{info.DeviceInformation.Id}");
                    Debug.WriteLine("");
                }
                Debug.WriteLine("-------------------------------------------------------------\n");

            }
        }

        public SpatialCoordinateSystem GetSpatialCoordinate()
        {
            return m_mediaInfo.CoordinateSystem;
        }

        /// <summary>
        /// Asynchronously create a Hand Detector with the first depth camera found
        /// </summary>
        /// <param name="id">The ID of the device to look for if needed. If NULL, a device with "depth" capabilities will be randomly choose.</param>
        /// <returns>The asynchronous task</returns>
        public static async Task<HandDetector> CreateAsync(String id=null)
        {
            //Search for the correct media frame source
            MediaFrameSourceGroup selectedFrameSourceGroup = null;
            MediaFrameSourceInfo  selectedFrameSourceInfo  = null;

            IReadOnlyList<MediaFrameSourceGroup> allFrameSourceGroups = await MediaFrameSourceGroup.FindAllAsync();

            foreach(MediaFrameSourceGroup group in allFrameSourceGroups)
            {
                foreach(MediaFrameSourceInfo info in group.SourceInfos)
                {
                    //If an ID is given
                    if((id == null || info.DeviceInformation.Id == id) && (info.MediaStreamType == MediaStreamType.VideoPreview || info.MediaStreamType == MediaStreamType.VideoRecord))
                    {
                        //Check the depth capabilities
                        if (info.SourceKind == MediaFrameSourceKind.Depth)
                        {                            
                            selectedFrameSourceGroup = group;
                            selectedFrameSourceInfo = info;

                            Debug.WriteLine($"Found Device : {info.DeviceInformation.Name}:{info.DeviceInformation.Id}");
                        }
                    }

                    if (selectedFrameSourceGroup != null)
                        break;
                }
                if(selectedFrameSourceGroup != null)
                    break;
            }

            if (selectedFrameSourceGroup == null)
            {
                Debug.WriteLine("No frame source available found");
                return null;
            }

            HandDetector HandDetector = new HandDetector(selectedFrameSourceGroup, selectedFrameSourceInfo);
            return HandDetector;
        }

        /// <summary>
        /// Creates asynchronously the Media Capture which will process the depth stream images
        /// </summary>
        /// <param name="clbk">The Callback object to call when the hand detection status changes.</param>
        /// <returns>The asynchronous task</returns>
        public async Task InitializeAsync(IHDMediaSinkClbk clbk)
        {
            //Create the media capture
            Debug.WriteLine("Creating a media capture...");
            m_mediaCapture = new MediaCapture();
            await m_mediaCapture.InitializeAsync(new MediaCaptureInitializationSettings()
            {
                SourceGroup             = m_mediaGroup,
                SharingMode             = MediaCaptureSharingMode.SharedReadOnly,
                MemoryPreference        = MediaCaptureMemoryPreference.Auto,   //For the Hololens, MediaCaptureMemoryPreference.CPU does not work
                StreamingCaptureMode    = StreamingCaptureMode.Video
            });
            
            //Find a correct video profile with the best capabilities (resolution)
            Debug.WriteLine("Search a video profile...");
            VideoEncodingProperties videoProfile = null;
            var mediaProperties = m_mediaCapture.VideoDeviceController.GetAvailableMediaStreamProperties(MediaStreamType.VideoPreview);
            UInt32 maxHeight = 0;

            foreach(var mediaProp in mediaProperties)
            {
                VideoEncodingProperties videoProp = mediaProp as VideoEncodingProperties;
                Debug.WriteLine($"VideoProp : {videoProp.Type}:{videoProp.Subtype}");
                if(videoProp.Subtype == "ARGB32" || videoProp.Subtype == "L8" || videoProp.Subtype == "D16" || videoProp.Subtype == "D8" || videoProp.Subtype == "L16" || videoProp.Subtype == "RGB24")
                { 
                    if(maxHeight < videoProp.Height)
                    { 
                        videoProfile = videoProp;
                        maxHeight = videoProp.Height;
                    }
                }
            }

            if (videoProfile == null)
            {
                Debug.WriteLine("No video profile found...");
                await Task.FromResult<Windows.Foundation.IAsyncAction>(null);
            }

            else
            {
                Debug.WriteLine($"Starting to preview {m_mediaInfo.DeviceInformation.Name} : {m_mediaInfo.DeviceInformation.Id} at {videoProfile.Width}x{videoProfile.Height}: {videoProfile.Subtype}");

                //Create the video encoding
                MediaEncodingProfile profile = new MediaEncodingProfile();
                profile.Video = videoProfile;
                profile.Audio = null;
                
                //Create and start preview in the MediaSink
                Debug.WriteLine(m_mediaInfo.DeviceInformation.Name);
                m_mediaSink = new HDMediaSinkProxy();
                IMediaExtension ext = await m_mediaSink.InitializeAsync(clbk, profile.Video);
                await m_mediaCapture.StartPreviewToCustomSinkAsync(profile, ext);
            }

            Debug.WriteLine("End of Create media capture async");
        }
    }
}