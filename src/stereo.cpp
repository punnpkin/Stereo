#include<iostream>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_node");
    
    int video_device;
    int frame_rate;

    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_left = it.advertise("left/image_raw", 10);
    image_transport::Publisher pub_right = it.advertise("right/image_raw", 10);

    nh.param<int>("video_device", video_device, 0);
    nh.param<int>("frame_rate", frame_rate, 30);

    cv::VideoCapture Stereo;
    Stereo.open(video_device);
    if(!Stereo.isOpened()){
        ROS_WARN("OPEN DEVICE %d ERROR!", video_device);
        return 0;
    }
    ROS_INFO("Camera device%d openned, fps=%d", video_device, frame_rate);

    ros::Rate loop_rate(frame_rate);
    while(nh.ok()){
        if(Stereo.grab()){
            cv::Mat image, image_left, image_right;
            Stereo.retrieve(image);

            cv::Rect rect_left(0,0,319,240); //(x,y,w,h)
            cv::Rect rect_right(320,0,319,240); //(x,y,w,h)
            image_left = cv::Mat(image, rect_left).clone();
            image_right = cv::Mat(image, rect_right).clone();

            cv_bridge::CvImage out_msg_left, out_msg_right;
            out_msg_left.header.stamp = out_msg_right.header.stamp = ros::Time::now();
            out_msg_left.encoding = out_msg_right.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg_left.image = image_left;
            out_msg_right.image = image_right;

            pub_left.publish(out_msg_left.toImageMsg());
            pub_right.publish(out_msg_right.toImageMsg());
        }
        loop_rate.sleep();
    }
    return 0;
}
