#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>

class laneMarker{
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber rawSubscriber;
        image_transport::Publisher debug_gray;
        image_transport::Publisher debug_mask;
        image_transport::Publisher debug_Gaussian;
        image_transport::Publisher debug_CannyEdge;
        image_transport::Publisher debug_Hough;
        image_transport::Publisher resultOverlay;
        image_transport::Publisher result;
        ros::Publisher lane_publisher;

        /*
        * This function is useful for publishing an image. It takes in a publisher and an cv::Mat image
        */
        void publishOpenCVImage(image_transport::Publisher publisher, cv::Mat inputImage, bool option){
            //cv_bridge::CvImage(header, encoding, image)
            cv::Mat displayImage;
            if (option){
                cv::cvtColor(inputImage,displayImage,CV_GRAY2BGR);
                ROS_WARN("Get BW image converted!");
            }else{
                displayImage = inputImage;
                ROS_WARN("Get BW image Not!");
            }
            cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, displayImage);
            sensor_msgs::Image imageMessage;
            imgBridge.toImageMsg(imageMessage);
            publisher.publish(imageMessage);
        }
        
    public:
        //* Constructor

        laneMarker() : it(nh) {
            //! Subscriber subscribe and then process the image using process
            rawSubscriber = it.subscribe("/front_realSense/color/image_raw", 1, &laneMarker::processIMG, this);
            debug_Gaussian = it.advertise("/debug/front_realSense/Gaussian",1);
            debug_CannyEdge = it.advertise("/debug/front_realSense/Cannyedge",1);
            debug_Hough = it.advertise("/debug/front_realSense/Houghtransform",1);
            debug_mask = it.advertise("/debug/front_realSense/MaskPure",1);

            resultOverlay = it.advertise("/front_realSense/cv_processed/canny_overlay",1);
            result = it.advertise("/front_realSense/cv_processed/result",1);

            lane_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/front_realSense/detected_lane",1);
        }


        void processIMG(const sensor_msgs::ImageConstPtr& srcImg){
            //! 0. Process Magic Variable (Can be replaced with dynamic reconfigure later)
            //* Gaussain Blur:
            cv::Size gaussian_Size = cv::Size(9,9);
            double gaussian_SigmaX = 0;
            double gaussian_Sigmay = 0;

            //* Canny Edge:
            double canny_lowThreashold = 100;
            double canny_highThreashold = 150;
            double canny_kernalSize = 3;

            ROS_INFO("Image Update Recieved!");
            //! 1. Read in image
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(srcImg,sensor_msgs::image_encodings::BGR8);
            }
            catch(const cv_bridge::Exception& e)
            {   
                ROS_ERROR("cv_bridge BGR converstion exception: %s",e.what());
                return;
            }
            cv::Mat processed_IMG;
            ROS_INFO("Image Read In Complete!");

            //! 2. Binarize and isolate out useless information
            cv::Mat mask = cv::Mat::zeros(processed_IMG.rows,processed_IMG.cols,CV_8U);
            //* Binary Mask Keypoints:
            cv::Point triangle[3] = {
                cv::Point(0,processed_IMG.rows),
                cv::Point(0,processed_IMG.cols/2),
                cv::Point(processed_IMG.rows,processed_IMG.cols)};
            cv::fillConvexPoly( mask, triangle, 3, cv::Scalar(1) );
            cv::bitwise_and(processed_IMG,processed_IMG,mask=mask);
            publishOpenCVImage(debug_mask,processed_IMG,true);

            //! 3. Convert into Gray Scale:
            cv::cvtColor(cv_ptr->image,processed_IMG,CV_BGR2GRAY);
            publishOpenCVImage(debug_gray,processed_IMG,true);
            ROS_INFO("Image Grayscale Conversion Complete!");

            //! 4. Gaussian Blur:
            /**
             * @param src input image
             * @param dst output image of the same size and type as src.
             * @param ksize Gaussian kernel size.
             * @param sigmaX Gaussian kernel standard deviation in X direction
             * @param sigmaY Gaussian kernel standard deviation in Y direction
             * @param borderType pixel extrapolation method
             * 
             * CV_EXPORTS_W void GaussianBlur( InputArray src, OutputArray dst, Size ksize,
                                double sigmaX, double sigmaY = 0,
                                int borderType = BORDER_DEFAULT );
             */
            cv::GaussianBlur(processed_IMG,processed_IMG,gaussian_Size,gaussian_SigmaX,gaussian_Sigmay);
            publishOpenCVImage(debug_Gaussian,processed_IMG,true);
            ROS_INFO("Gaussian Blur Complete!");

            //! 5. Canny Edge Detector
            cv::Canny(processed_IMG,processed_IMG,canny_lowThreashold, canny_highThreashold, canny_kernalSize);
            publishOpenCVImage(debug_CannyEdge,processed_IMG,true);
            ROS_INFO("Canny Edge Complete!");

            //! 6. Hough transform
        }

        void createMask(cv::Mat &frame){
            int rows = frame.rows;
            int cols = frame.cols;

            cv::Point points[1][4];
            points[0][0] = cv::Point(cols*0.05, rows);
            points[0][1] = cv::Point(cols*0.4, rows*0.4);
            points[0][2] = cv::Point(cols*0.6, rows*0.4);
            points[0][3] = cv::Point(cols*0.95, rows);


        };
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "laneMarker");
    laneMarker worker;
    ros::spin();    
    return 0;
}