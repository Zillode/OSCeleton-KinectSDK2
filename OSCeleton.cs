using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using Microsoft.Samples.Kinect.FaceBasics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Ventuz.OSC;
using OSCeleton;

namespace OSCeleton
{
    class OSCeleton
    {

        // Settings
        private bool allUsers = true;
        private bool fullBody = true;
        private bool faceTracking = false;
        private bool writeOSC = true;
        private bool writeCSV = true;
        private bool useUnixEpochTime = true;
        private String oscHost = "127.0.0.1";
        private int oscPort = 7110;
        private const int skeletonCount = 6;
        private const int pointScale = 1000;

        // Outputs
        private bool capturing = true;
        private BlockingCollection<TrackingInformation> trackingInformationQueue = new BlockingCollection<TrackingInformation>();
        Thread sendTracking;
        private UdpWriter osc;
        private StreamWriter fileWriter;
        private Stopwatch stopwatch;

        public void Initialise()
        {
            // Install Shortcut
            CheckForShortcut();

            // Parse commandline arguments
            string[] args = Environment.GetCommandLineArgs();
            for (int index = 1; index < args.Length; index += 2)
            {
                args[index] = args[index].ToLower();
                if ("allUsers".ToLower().Equals(args[index])) allUsers = StringToBool(args[index + 1]);
                if ("fullBody".ToLower().Equals(args[index])) fullBody = StringToBool(args[index + 1]);
                if ("faceTracking".ToLower().Equals(args[index])) faceTracking = StringToBool(args[index + 1]);
                if ("writeOSC".ToLower().Equals(args[index])) writeOSC = StringToBool(args[index + 1]);
                if ("writeCSV".ToLower().Equals(args[index])) writeCSV = StringToBool(args[index + 1]);
                if ("useUnixEpochTime".ToLower().Equals(args[index])) useUnixEpochTime = StringToBool(args[index + 1]);
                if ("oscHost".ToLower().Equals(args[index])) oscHost = args[index + 1];
                if ("oscPort".ToLower().Equals(args[index]))
                {
                    if (!int.TryParse(args[index + 1], out oscPort))
                    {
                        System.Windows.MessageBox.Show("Failed to parse the oscPort argument: " + args[index + 1]);
                    }
                }
            }

            // Initialisation
            stopwatch = new Stopwatch();
            stopwatch.Reset();
            stopwatch.Start();
            if (writeOSC)
            {
                osc = new UdpWriter(oscHost, oscPort);
            }
            if (writeCSV)
            {
                OpenNewCSVFile();
            }
            if (sendTracking == null)
            {
                sendTracking = new Thread(SendTrackingInformation);
                sendTracking.Start();
            }
        }

        public void Stop()
        {
            capturing = false;
            if (sendTracking != null)
            {
                sendTracking.Abort();
                sendTracking = null;
            }
        }

        public void OpenNewCSVFile()
        {
            if (!writeCSV) return;

            CloseCSVFile();
            StreamWriter fileWriter = InitCSVFile();
            if (this.fileWriter != null)
            {
                lock (fileWriter)
                {
                    this.fileWriter = fileWriter;
                }
            }
            else
            {
                this.fileWriter = fileWriter;
            }
        }

        public StreamWriter InitCSVFile()
        {
            StreamWriter fileWriter = new StreamWriter(Environment.GetFolderPath(Environment.SpecialFolder.Personal) + "/points-MSK2-" + getUnixEpochTime().ToString().Replace(",", ".") + ".csv", false);
            fileWriter.WriteLine("Joint, sensor, user, joint, x, y, z, confidence, time");
            fileWriter.WriteLine("Face, sensor, user, pitch, yaw, roll, time");
            fileWriter.WriteLine("FaceProperty, sensor, user, happy, engaged, wearingGlasses, leftEyeClosed, rightEyeClosed, mouthOpen, mouthMoved, lookingAway, time");
            return fileWriter;
        }

        private void CloseCSVFile()
        {
            if (fileWriter != null)
            {
                lock (fileWriter)
                {
                    if (fileWriter != null)
                    {
                        fileWriter.Close();
                        fileWriter = null;
                    }
                }
            }
        }

        public void CheckForShortcut()
        {
            try
            {
                if (System.Diagnostics.Debugger.IsAttached)
                {
                    return;
                }
                System.Deployment.Application.ApplicationDeployment ad = default(System.Deployment.Application.ApplicationDeployment);
                ad = System.Deployment.Application.ApplicationDeployment.CurrentDeployment;

                if ((ad.IsFirstRun))
                {
                    System.Reflection.Assembly code = System.Reflection.Assembly.GetExecutingAssembly();
                    string company = string.Empty;
                    string description = string.Empty;

                    if ((Attribute.IsDefined(code, typeof(System.Reflection.AssemblyCompanyAttribute))))
                    {
                        System.Reflection.AssemblyCompanyAttribute ascompany = null;
                        ascompany = (System.Reflection.AssemblyCompanyAttribute)Attribute.GetCustomAttribute(code, typeof(System.Reflection.AssemblyCompanyAttribute));
                        company = ascompany.Company;
                    }

                    if ((Attribute.IsDefined(code, typeof(System.Reflection.AssemblyTitleAttribute))))
                    {
                        System.Reflection.AssemblyTitleAttribute asdescription = null;
                        asdescription = (System.Reflection.AssemblyTitleAttribute)Attribute.GetCustomAttribute(code, typeof(System.Reflection.AssemblyTitleAttribute));
                        description = asdescription.Title;

                    }

                    if ((company != string.Empty & description != string.Empty))
                    {
                        //description = Replace(description, "_", " ")

                        string desktopPath = string.Empty;
                        desktopPath = string.Concat(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "\\", description, ".appref-ms");

                        string shortcutName = string.Empty;
                        shortcutName = string.Concat(Environment.GetFolderPath(Environment.SpecialFolder.Programs), "\\", company, "\\", description, ".appref-ms");

                        System.IO.File.Copy(shortcutName, desktopPath, true);
                    }
                    else
                    {
                        System.Windows.MessageBox.Show("Missing company or description: " + company + " - " + description);
                    }
                }
            }
            catch (Exception ex)
            {
                System.Windows.MessageBox.Show(GetErrorText(ex));
            }
        }

        public void EnqueueBody(int sensorId, int user, Body b)
        {
            if (!capturing) { return; }
            if (b == null) { return; }
            trackingInformationQueue.Add(new BodyTrackingInformation(sensorId, user, b, fullBody, getTime()));
        }

        double detectionResultToConfidence(DetectionResult r)
        {
            switch (r)
            {
                case DetectionResult.Unknown:
                    return 0.5;
                case DetectionResult.Maybe:
                    return 0.5;
                case DetectionResult.No:
                    return 0;
                case DetectionResult.Yes:
                    return 1;
                default:
                    return 0.5;
            }
        }

        public void EnqueueFaceTracking(int sensorId, int user, Microsoft.Kinect.Face.FaceFrameResult faceResult)
        {
            if (!capturing) { return; }
            if (faceResult == null) { return; }

            
            // extract face rotation in degrees as Euler angles
            if (faceResult.FaceRotationQuaternion != null)
            {
                int pitch, yaw, roll;
                MainWindow.ExtractFaceRotationInDegrees(faceResult.FaceRotationQuaternion, out pitch, out yaw, out roll);
                trackingInformationQueue.Add(new FaceRotationTrackingInformation(sensorId, user, pitch, yaw, roll, getTime()));
            }

            // extract each face property information and store it in faceText
            if (faceResult.FaceProperties != null)
            {
                trackingInformationQueue.Add(new FacePropertyTrackingInformation(sensorId, user,
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.Happy]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.Engaged]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.WearingGlasses]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.LeftEyeClosed]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.RightEyeClosed]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.MouthOpen]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.MouthMoved]),
                    detectionResultToConfidence(faceResult.FaceProperties[FaceProperty.LookingAway]),
                    getTime()));
            }
        }

        void SendTrackingInformation()
        {
            while (true)
            {
                TrackingInformation i = trackingInformationQueue.Take();
                if (i != null && capturing)
                lock (fileWriter)
                {
                        i.Send(osc, fileWriter, pointScale);
                }
            }
        }

        private double getTime()
        {
            if (useUnixEpochTime)
                return getUnixEpochTime();
            return stopwatch.ElapsedMilliseconds;
        }

        private double getUnixEpochTime()
        {
            var unixTime = DateTime.Now.ToUniversalTime() - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            return unixTime.TotalMilliseconds;
        }

        private long getUnixEpochTimeLong()
        {
            var unixTime = DateTime.Now.ToUniversalTime() - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            return System.Convert.ToInt64(unixTime.TotalMilliseconds);
        }

        private string GetErrorText(Exception ex)
        {
            string err = ex.Message;
            if (ex.InnerException != null)
            {
                err += " - More details: " + ex.InnerException.Message;
            }
            return err;
        }

        bool StringToBool(String msg)
        {
            msg = msg.ToLower();
            return msg.Equals("1") || msg.ToLower().Equals("true");
        }
    }
}
