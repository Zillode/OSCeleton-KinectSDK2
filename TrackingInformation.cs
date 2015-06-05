using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Ventuz.OSC;

namespace OSCeleton
{
    abstract public class TrackingInformation
    {
        public int sensorId;
        public abstract void Send(UdpWriter osc, StreamWriter fileWriter, int pointScale);
    }

    public class BodyTrackingInformation : TrackingInformation {

        public int user;
        public Body body;
        public bool fullBody;
        public double time;

        public static List<String> oscMapping = new List<String> { "",
            "head", "neck", "torso", "waist",
            "l_collar", "l_shoulder", "l_elbow", "l_wrist", "l_hand", "l_fingertip",
            "r_collar", "r_shoulder", "r_elbow", "r_wrist", "r_hand", "r_fingertip",
            "l_hip", "l_knee", "l_ankle", "l_foot",
            "r_hip", "r_knee", "r_ankle", "r_foot" };

        public BodyTrackingInformation(int sensorId, int user, Body body, bool fullBody, double time)
        {
            this.sensorId = sensorId;
            this.user = user;
            this.body = body;
            this.fullBody = fullBody;
            this.time = time;
        }

        public override void Send(UdpWriter osc, StreamWriter fileWriter, int pointScale)
        {
            if (body == null) return;
            if (body.Joints == null || body.Joints.Count < 20) return;
            if (body.JointOrientations == null || body.JointOrientations.Count < 20) return;
            if (!body.IsTracked) return;
            if (!fullBody)
            {
                ProcessJointInformation(15, body.Joints[JointType.HandRight], body.JointOrientations[JointType.HandRight], time, osc, fileWriter, pointScale);
            }
            else
            {
                ProcessJointInformation(1, body.Joints[JointType.Head], body.JointOrientations[JointType.Head], time, osc, fileWriter, pointScale);
                ProcessJointInformation(2, body.Joints[JointType.SpineShoulder], body.JointOrientations[JointType.SpineShoulder], time, osc, fileWriter, pointScale);
                ProcessJointInformation(3, body.Joints[JointType.SpineMid], body.JointOrientations[JointType.SpineMid], time, osc, fileWriter, pointScale);
                ProcessJointInformation(4, body.Joints[JointType.SpineBase], body.JointOrientations[JointType.SpineBase], time, osc, fileWriter, pointScale);
                // ProcessJointInformation(5, body.Joints[JointType.], body.JointOrientations[JointType.], time, osc, fileWriter, pointScale);
                ProcessJointInformation(6, body.Joints[JointType.ShoulderLeft], body.JointOrientations[JointType.ShoulderLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(7, body.Joints[JointType.ElbowLeft], body.JointOrientations[JointType.ElbowLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(8, body.Joints[JointType.WristLeft], body.JointOrientations[JointType.WristLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(9, body.Joints[JointType.HandLeft], body.JointOrientations[JointType.HandLeft], time, osc, fileWriter, pointScale);
                // ProcessJointInformation(10, body.Joints[JointType.], body.JointOrientations[JointType.], time, osc, fileWriter, pointScale);
                // ProcessJointInformation(11, body.Joints[JointType.], body.JointOrientations[JointType.], time, osc, fileWriter, pointScale);
                ProcessJointInformation(12, body.Joints[JointType.ShoulderRight], body.JointOrientations[JointType.ShoulderRight], time, osc, fileWriter, pointScale);
                ProcessJointInformation(13, body.Joints[JointType.ElbowRight], body.JointOrientations[JointType.ElbowRight], time, osc, fileWriter, pointScale);
                ProcessJointInformation(14, body.Joints[JointType.WristRight], body.JointOrientations[JointType.WristRight], time, osc, fileWriter, pointScale);
                ProcessJointInformation(15, body.Joints[JointType.HandRight], body.JointOrientations[JointType.HandRight], time, osc, fileWriter, pointScale);
                // ProcessJointInformation(16, body.Joints[JointType.], body.JointOrientations[JointType.], time, osc, fileWriter, pointScale);
                ProcessJointInformation(17, body.Joints[JointType.HipLeft], body.JointOrientations[JointType.HipLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(18, body.Joints[JointType.KneeLeft], body.JointOrientations[JointType.KneeLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(19, body.Joints[JointType.AnkleLeft], body.JointOrientations[JointType.AnkleLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(20, body.Joints[JointType.FootLeft], body.JointOrientations[JointType.FootLeft], time, osc, fileWriter, pointScale);
                ProcessJointInformation(21, body.Joints[JointType.HipRight], body.JointOrientations[JointType.HipRight], time, osc, fileWriter, pointScale);
                ProcessJointInformation(22, body.Joints[JointType.KneeRight], body.JointOrientations[JointType.KneeRight], time, osc, fileWriter, pointScale);
                ProcessJointInformation(23, body.Joints[JointType.AnkleRight], body.JointOrientations[JointType.AnkleRight], time, osc, fileWriter, pointScale);
                ProcessJointInformation(24, body.Joints[JointType.FootRight], body.JointOrientations[JointType.FootRight], time, osc, fileWriter, pointScale);
            }
        }


        double JointToConfidenceValue(Joint j)
        {
            if (j.TrackingState == TrackingState.Tracked) return 1;
            if (j.TrackingState == TrackingState.Inferred) return 0.5;
            if (j.TrackingState == TrackingState.NotTracked) return 0.1;
            return 0.5;
        }

        void ProcessJointInformation(int joint, Joint j, JointOrientation jo, double time, UdpWriter osc, StreamWriter fileWriter, int pointScale)
        {
            SendJointMessage(joint,
                j.Position.X, j.Position.Y, j.Position.Z,
                JointToConfidenceValue(j), time,
                osc, fileWriter, pointScale);
        }

        void SendJointMessage(int joint, double x, double y, double z, double confidence, double time, UdpWriter osc, StreamWriter fileWriter, int pointScale)
        {
            if (osc != null)
            {
                osc.Send(new OscElement("/osceleton2/joint", oscMapping[joint], sensorId, user, (float)(x * pointScale), (float)(-y * pointScale), (float)(z * pointScale), (float)confidence, time));
            }
            if (fileWriter != null)
            {
                // Joint, user, joint, x, y, z, on
                fileWriter.WriteLine("Joint," + sensorId + "," + user + "," + joint + "," +
                    (x * pointScale).ToString().Replace(",", ".") + "," +
                    (-y * pointScale).ToString().Replace(",", ".") + "," +
                    (z * pointScale).ToString().Replace(",", ".") + "," +
                    confidence.ToString().Replace(",", ".") + "," +
                    time.ToString().Replace(",", "."));
            }
        }
    }

    public class FaceRotationTrackingInformation : TrackingInformation {
        public int user;
        public int pitch, yaw, roll;
        public double time;

        public FaceRotationTrackingInformation(int sensorId, int user, int pitch, int yaw, int roll, double time)
        {
            this.sensorId = sensorId;
            this.user = user;
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.time = time;
        }

        public override void Send(UdpWriter osc, StreamWriter fileWriter, int pointScale)
        {
            if (osc != null)
            {
                osc.Send(new OscElement(
                    "/osceleton2/face_rotation",
                    sensorId, user,
                    pitch, yaw, roll,
                    time));
            }
            if (fileWriter != null)
            {
                fileWriter.WriteLine("FaceRotation," +
                    sensorId + "," + user + "," +
                    pitch + "," + yaw + "," + roll + "," +
                    time.ToString().Replace(",", "."));
            }
        }
    }
    
    public class FacePropertyTrackingInformation : TrackingInformation {
        int user;
        float happy, engaged, wearingGlasses, leftEyeClosed, rightEyeClosed, mouthOpen, mouthMoved, lookingAway;
        double time;
        public FacePropertyTrackingInformation(int sensorId, int user, float happy, float engaged, float wearingGlasses,
            float leftEyeClosed, float rightEyeClosed, float mouthOpen, float mouthMoved, float lookingAway, double time) {
            this.sensorId = sensorId;
            this.user = user;
            this.happy = happy;
            this.engaged = engaged;
            this.wearingGlasses = wearingGlasses;
            this.leftEyeClosed = leftEyeClosed;
            this.rightEyeClosed = rightEyeClosed;
            this.mouthOpen = mouthOpen;
            this.mouthMoved = mouthMoved;
            this.lookingAway = lookingAway;
            this.time = time;
        }
        
            public override void Send(UdpWriter osc, StreamWriter fileWriter, int pointScale)
            {
                if (osc != null)
                {
                    osc.Send(new OscElement(
                        "/osceleton2/face_property",
                        sensorId, user,
                        happy, engaged, wearingGlasses, leftEyeClosed, rightEyeClosed, mouthOpen, mouthMoved, lookingAway,
                        time));
                }
                if (fileWriter != null)
                {
                    fileWriter.WriteLine("FaceProperty," +
                        sensorId + "," + user + "," +
                        happy.ToString().Replace(",", ".") + "," +
                        engaged.ToString().Replace(",", ".") + "," +
                        wearingGlasses.ToString().Replace(",", ".") + "," +
                        leftEyeClosed.ToString().Replace(",", ".") + "," +
                        rightEyeClosed.ToString().Replace(",", ".") + "," +
                        mouthOpen.ToString().Replace(",", ".") + "," +
                        mouthMoved.ToString().Replace(",", ".") + "," +
                        lookingAway.ToString().Replace(",", ".") + "," +
                        time.ToString().Replace(",", "."));
                }
            }
    }
}
