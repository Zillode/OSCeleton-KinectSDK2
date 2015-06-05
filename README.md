OSCeleton-KinectSDK2
====================

What is this?
-------------

As the title says, it's just a small program that takes kinect
skeleton data from the KinectSDK (v2) framework and spits out the coordinates
of the skeleton's joints via OSC messages. These can can then be used
on your language / framework of choice.

This version of OSCeleton is similar to [OSCeleton-OpenNI](http://github.com/Zillode/OSCeleton-OpenNI) and [OSCeleton-KinectSDK](http://github.com/Zillode/OSCeleton-KinectSDK).

How do I use it?
----------------

### Install Visual Studio 2013
### Install [Microsoft Kinect SDK (version 2.0)](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
### Install [Microsoft Speech Platform SDK (version 11)](http://www.microsoft.com/en-us/download/details.aspx?id=27226)
### Compile and run the OSCeleton-KinectSDK2 solution

If you run the executable, it will send the OSC
messages in the Midas format to localhost on port 7110.
To learn about the OSC message format, continue reading below.


OSC Message format
------------------

### Joint message - message with the coordinates of each skeleton joint:
The messages will have the following format:

    Address pattern: "/osceleton2/joint"
    Type tag: "siiffffd"
    s: Joint name, check out the full list of joints below
    i: The ID of the sensor
    i: The ID of the user
    f: X coordinate of joint in real world coordinates (centimers)
    f: Y coordinate of joint in real world coordinates (centimers)
    f: Z coordinate of joint in real world coordinates (centimers)
    f: confidence value in interval [0.0, 1.0]
	d: timestamp in milliseconds since launch

Note: the Y coordinate is inverted compared to the default KinectSDK to be compatible with OpenNI.

### FaceRotation message - message with the rotation coordinates of a face event:
The messages will have the following format:

    Address pattern: "/osceleton2/face_rotation"
    Type tag: "iiffffd"
    i: The ID of the sensor
    i: The ID of the user
    f: pitch of the head [-90, 90]
    f: pitch of the yaw [-90, 90]
    f: pitch of the roll [-90, 90]
	d: timestamp in milliseconds since launch
	
Further information about the FaceRotation properties can be found [here](https://msdn.microsoft.com/en-us/library/microsoft.kinect.face.faceframeresult.facerotationquaternion.aspx)

### FaceProperty message - message with the coordinates of a face event:
The messages will have the following format:

    Address pattern: "/osceleton2/face_property"
    Type tag: "iiffffd"
    i: The ID of the sensor
    i: The ID of the user
    f: happy [0, 1]
    f: engaged [0, 1]
    f: wearing glasses [0, 1]
    f: left eye closed [0, 1]
    f: right eye closed [0, 1]
    f: mouth open [0, 1]
    f: mouth moved [0, 1]
    f: looking away [0, 1]
	d: timestamp in milliseconds since launch

Further information about the Face properties can be found [here](https://msdn.microsoft.com/en-us/library/microsoft.kinect.face.faceproperty.aspx)


### Full list of joints

* head -> Head
* neck -> SpineShoulder
* torso -> SpineMid
* waist -> SpineBase
* r_collar #not supported by KinectSDK (yet)
* r_shoulder -> ShoulderRight
* r_elbow -> ElbowRight
* r_wrist -> WristRight
* r_hand -> HandRight
* r_finger #not supported by KinectSDK (yet)
* l_collar #not supported by KinectSDK (yet)
* l_shoulder -> ShoulderLeft
* l_elbow -> ElbowLeft
* l_wrist -> WristLeft
* l_hand -> HandLeft
* l_finger #not supported by KinectSDK (yet)
* r_hip -> HipRight
* r_knee -> KneeRight
* r_ankle -> AnkleRight
* r_foot -> FootRight
* l_hip -> HipLeft
* l_knee -> KneeLeft
* l_ankle -> AnkleLeft
* l_foot -> FootLeft


Other
-----
### For feature request, reporting bugs, or general OSCeleton 
discussion, come join the fun in a related [google group](http://groups.google.com/group/osceleton)!

### OSCeleton-OpenNI ?
To use the OpenNI & NITE framework in combination with OSC messages, download [OSCeleton-OpenNI](https://github.com/Zillode/OSCeleton-OpenNI)

### OSCeleton-KinectSDK ?
To use the Kinect SDK v1.8 in combination with OSC messages, download [OSCeleton-KinectSDK](https://github.com/Zillode/OSCeleton-KinectSDK)

Have fun!


