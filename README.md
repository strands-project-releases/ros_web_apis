# ros_web_apis
Some WEB APIs for ROS usage

## ms_face_api

This is a ROS wrapper for the [Microsoft Cognitive Services FACE API](https://www.microsoft.com/cognitive-services/en-us/face-api). It offers ROS services to detect faces, extract face markers, and recognise attributes like "gender", "smile", and "age". Also, faces can be trained for identification and if a face is trained, the detection will also contain the identity of a person in the image.

### Services:

#### Detection and optional Identification

* call service `cognitivefaceapi/detect` to detect faces in an image
* Service Definition: [`ms_face_api/Detect`](/LCAS/ros_web_apis/blob/master/ms_face_api/srv/Detect.srv)
  * An image can either be defined by defining in the requests 
    1. a `sensor_msgs/Image`,
    2. a `filename`, or
    3. a `topic` where the image is read from (waiting for a timeout)
  * It returns an [`ms_face_api/Faces`](https://github.com/LCAS/ros_web_apis/blob/master/ms_face_api/msg/Faces.msg) data structure
  * if the `identify` flag is `true` in the request, the API tries to identify all the detected faces

#### Face Training

* call service `cognitivefaceapi/add_face` to add the largest face in the provided image with a given `name`
* Service Definition: [`ms_face_api/AddIdentityFace`](/LCAS/ros_web_apis/blob/master/ms_face_api/srv/AddIdentityFace.srv)
  * An image is defined in the same way as for `Detect` (se above)
* The person is given a name in the request in the `person` field. If this person is already known, then the image is added to the training set for that person. If the person is not known, a new person is created and the face added to the training set.
* This service immediately retrains the face identifications system.
* Afterwards, the trained person should be identified by the `Detect` service (potentially after some delay as training can take some time)
