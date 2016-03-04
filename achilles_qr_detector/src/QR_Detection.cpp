#include"QR_Detection.h"

QR_Detection::QR_Detection()
{

    ros::NodeHandle n("~");

    n.param( "camera_index_1", id_camera, 10);
    //    n.param( "/forwad/camera_index_2", id_camera_2, 2);
    //    n.param( "/forwad/camera_index_2", id_camera_3, 3);
    n.param( "show", show , false );
    n.param( "fps", fps , 60 );


    //    if(QFile::exists(VICTIM_CAMERA_ID)==false)
    //    {
    //        ROS_ERROR("The Video Doesn't Exits...");
    //        ros::shutdown();
    //    }

    //    QProcess camera_id;
    //    camera_id.start("ls", QStringList() << "-l" << VICTIM_CAMERA_ID);
    //    if (!camera_id.waitForStarted(6000))
    //        return ;

    //    if (!camera_id.waitForFinished(6000))
    //        return ;

    //    QByteArray result = camera_id.readAll();
    //    QString result_str=result.data();

    input_video.open(id_camera);
    //    input_video_2.open(id_camera_2);
    //    input_video_3.open(id_camera_3);

    //    input_video.set(CV_CAP_PROP_FPS,60);
    //    input_video_2.set(CV_CAP_PROP_FPS,60);

    //    id_camera = QString(result_str[result_str.indexOf("->")+8]).toInt();

    pub = n.advertise<achilles_qr_detector::qrPos>("Qr_String",10);
    //    cv_bridge::CvImage victim_image_msg_;

    victim_image_req_ = n.advertiseService("victim_image_req",&QR_Detection::victimImageReqCB,this);


    input_video.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    input_video.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    input_video.set(CV_CAP_PROP_FPS, 60);

    if(!input_video.isOpened())
    {
        ROS_ERROR("Couldn't Open The Camera !");
        ROS_ERROR("Babay :(");
        ros::shutdown();
    }
//    if(!input_video_2.isOpened())
//    {
//        ROS_ERROR("Couldn't Open The Camera !");
//        ROS_ERROR("Babay :(");
//        //        ros::shutdown();
//    }
    //    if(!input_video_3.isOpened())
    //    {
    //        ROS_ERROR("Couldn't Open The Camera !");
    //        ROS_ERROR("Babay :(");
    //        //        ros::shutdown();
    //    }
    //    msg.id = 1;


    ros::Rate loop_rate(fps);

    while (ros::ok())
    {
        //        ROS_ERROR("sassaa");
        input_video.read(frame);
        //        last_frame_=frame;
//        ros::spinOnce();
        this->QR_Decoder(frame,id_camera);
        //        imshow("imgout.jpg", frame);


//        input_video_2.read(frame);
        //        last_frame_=frame;
        //        ros::spinOnce();
//        this->QR_Decoder(frame,2);

        //        input_video_3.read(frame);

        //        this->QR_Decoder(frame,3);

//        ROS_ERROR("sassaa");
        loop_rate.sleep();

    }

}

bool QR_Detection::victimImageReqCB(achilles_qr_detector::VictimImage::Request &req, achilles_qr_detector::VictimImage::Response &res)
{
    ROS_ERROR("vitim image call back");
    cv_bridge::CvImage res_image;
    res_image.image = last_frame_;
    res_image.encoding = sensor_msgs::image_encodings::BGR8;
    res_image.toImageMsg(res.image_);
    return true;
}


Mat QR_Detection::QR_Decoder(Mat image,int id)
{
    msg.id = id;
    Mat img;
    width  = image.cols;
    height = image.rows;
    cvtColor(image,img,CV_RGB2GRAY);
    raw = (uchar *)img.data;
    Image image1(width, height, "Y800", raw, width * height);
    scanner.scan(image1);
    for(Image::SymbolIterator symbol = image1.symbol_begin(); symbol != image1.symbol_end(); ++symbol)
    {
        vector<Point> vp;
        cout << "decoded ";
        cout << "decoded " << symbol->get_type_name()<< " symbol \"" << symbol->get_data() << '"' <<" "<< endl;
        if(is_digits(symbol->get_data()) || (symbol->get_data().size() == 1))
        {

        }
        else
        {
            msg.context = symbol->get_data();
            pub.publish(msg);
        }
    }
    image1.set_data(NULL, 0);
    return image;

}
