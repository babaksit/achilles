#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "zbar.h"
#include "achilles_qr_detector/qrPos.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "achilles_qr_detector/VictimImage.h"
#include "QObject"
#include <QProcess>
#include <QFile>

#define CAMERA_ID "/dev/video58"

#define CAMERA_ID_2 "/dev/video59"


using namespace std;
using namespace zbar;
using namespace cv;

class QByteArray;
class QString;
class QProcess;
class QR_Detection : public QObject
{
public:

    int id_camera;
    int id_camera_2;
    int id_camera_3;
    int fps;
    bool show;
    int width, height;

    uchar *raw;
    ImageScanner scanner;
    VideoCapture input_video;
    VideoCapture input_video_2;
    VideoCapture input_video_3;
    Mat frame;
    Mat last_frame_;


    achilles_qr_detector::qrPos msg;

    ros::Publisher pub;

    explicit QR_Detection();

    image_transport::Publisher victim_image_pub_;
    ros::ServiceServer victim_image_req_;
    bool victimImageReqCB(achilles_qr_detector::VictimImage::Request &req,
                          achilles_qr_detector::VictimImage::Response &res);


private:
    bool is_digits(const std::string &str)
    {
        return str.find_first_not_of("0123456789") == std::string::npos;
    }
    Mat QR_Decoder(Mat image, int id);



};
