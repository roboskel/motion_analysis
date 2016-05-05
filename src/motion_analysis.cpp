/* Capture video input from camera and display on screen */

#include "cv.h"
#include "highgui.h"
#include "process.h"

#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "motion_analysis/shapes_msg.h"


cv_bridge::CvImagePtr image = 0;  
unsigned int IMAGE_RECEIVED = 0;
sensor_msgs::ImageConstPtr ros_image;
std_msgs::String string_msg;

void imageCallback(const sensor_msgs::Image::ConstPtr& str_img){
  if(IMAGE_RECEIVED == 0){
    ros_image = str_img;
    IMAGE_RECEIVED = 1;
  }
}

void objectStateCallback(const std_msgs::Int32 obj_state){
  placed = obj_state.data;
}

cv_bridge::CvImagePtr cvImageFromROS(){
  try{
    return cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return cv_bridge::CvImagePtr();
    }
}

int main(int argc, char** argv) {
  
  int x,y;
  unsigned char RGB[640][480][3];
  unsigned char RGB_unedited[640][480][3];
  unsigned char Bac[640][480][3];  

  ros::init(argc, argv, "motion_analysis");
  ros::NodeHandle n;
  std::string image_topic = "";
  std::string bounding_box_topic = "";
  std::string motion_detection_results_topic = "";
  std::string motion_analysis_shapes_topic = "";
  std::string motion_analysis_human_topic = "";
  std::string motion_analysis_object_topic = "";
  bool visualize = false;

  n.param("motion_analysis/image_topic", image_topic, std::string("/usb_cam/image_raw"));
  n.param("motion_analysis/bounding_box_topic", bounding_box_topic, std::string("/motion_analysis/bounding_box_viz"));
  n.param("motion_analysis/image_motion_detection_results_topic", motion_detection_results_topic, std::string("/motion_analysis/motion_analysis_viz"));
  n.param("motion_analysis/motion_analysis_shapes_topic", motion_analysis_shapes_topic, std::string("/motion_analysis/shapes_image"));
  n.param("motion_analysis/motion_analysis_human_topic", motion_analysis_human_topic, std::string("/motion_analysis/event/human_transfer"));
  n.param("motion_analysis/motion_analysis_object_topic", motion_analysis_object_topic, std::string("/motion_analysis/event/object_tampered"));
  n.param("motion_analysis/visualize", visualize, false);

  ros::Subscriber img_in, object_state_in;
  img_in = n.subscribe<sensor_msgs::Image>(image_topic, 5, imageCallback);
  object_state_in = n.subscribe<std_msgs::Int32>("object_state", 5, objectStateCallback);

  ros::Publisher string_publisher_person = n.advertise<std_msgs::String>(motion_analysis_human_topic, 1);
  ros::Publisher string_publisher_object = n.advertise<std_msgs::String>(motion_analysis_object_topic, 1);
  ros::Publisher shapes_image_publisher = n.advertise<motion_analysis::shapes_msg>(motion_analysis_shapes_topic, 1);
  ros::Publisher image_publisher_bb = n.advertise<sensor_msgs::Image>(bounding_box_topic, 100);
  ros::Publisher image_publisher_mdr = n.advertise<sensor_msgs::Image>(motion_detection_results_topic, 100);
  
  unsigned int iteration, index , showanno;
  
  showanno = 2;
  iteration = 1;
  std::string bed_answer;
  std::string obj_answer;

  motion_analysis::shapes_msg shapes_image_msg;
  int shapesxy[6] = {};


 while(ros::ok()) {
    if(IMAGE_RECEIVED == 1){
      if (iteration>16383) iteration=0; else iteration++;
    
    
      image  = cvImageFromROS();
      unsigned char *image_data = (unsigned char*)(image->image.data);

      cv_bridge::CvImagePtr unedited_image = cvImageFromROS();
      unsigned char *unedited_image_data = (unsigned char*)(unedited_image->image.data);
      
      index = iteration;
    
      int rows    = image->image.rows;
      int columns = image->image.cols;
      int step    = image->image.step;
      int channels= image->image.channels();
      
      for (y=0;y<rows;y++) {
          for (x=0;x<columns;x++) {
            RGB[639-x][y][REDV  ] = image_data[step*y+x*3+0];
            RGB[639-x][y][GREENV] = image_data[step*y+x*3+1];
            RGB[639-x][y][BLUEV ] = image_data[step*y+x*3+2];
            RGB_unedited[639-x][y][REDV] = unedited_image_data[step*y+x*3+0];
            RGB_unedited[639-x][y][GREENV] = unedited_image_data[step*y+x*3+1];
            RGB_unedited[639-x][y][BLUEV] = unedited_image_data[step*y+x*3+2];
          }
      }

      bed_answer = "";
      obj_answer = "";
      
      process((unsigned char *)RGB,(unsigned char *)Bac,index,showanno, bed_answer, obj_answer, (unsigned char *)RGB_unedited, shapesxy);
      
      if(placed == MODE_HUMAN_MOVEMENT){
        if(bed_answer.compare("") != 0){
          string_msg.data = bed_answer;
          string_publisher_person.publish(string_msg);
        }
      }
      else{
        if(obj_answer.compare("") != 0){
          string_msg.data = bed_answer;
          string_publisher_object.publish(string_msg);
        }
      }
      
      
      for (y=0;y<rows;y++) {
        for (x=0;x<columns;x++) {
          if(visualize){
            image_data[step*y+x*3+0] = RGB[x][y][REDV  ];
            image_data[step*y+x*3+1] = RGB[x][y][GREENV];
            image_data[step*y+x*3+2] = RGB[x][y][BLUEV ];
          }
          unedited_image_data[step*y+x*3+0] = RGB_unedited[x][y][REDV];
          unedited_image_data[step*y+x*3+1] = RGB_unedited[x][y][GREENV];
          unedited_image_data[step*y+x*3+2] = RGB_unedited[x][y][BLUEV];
        }
      }

      if(visualize){
        image_publisher_bb.publish(unedited_image->toImageMsg());
        image_publisher_mdr.publish(image->toImageMsg());
      }

      shapes_image_msg.boundingBox.top_left.x = shapesxy[0];
      shapes_image_msg.boundingBox.top_left.y = shapesxy[1];
      shapes_image_msg.boundingBox.bottom_right.x = shapesxy[2];
      shapes_image_msg.boundingBox.bottom_right.y = shapesxy[3];
      shapes_image_msg.centerOfActivity.x = shapesxy[4];
      shapes_image_msg.centerOfActivity.y = shapesxy[5];
      shapes_image_msg.image = *unedited_image->toImageMsg();

      shapes_image_publisher.publish(shapes_image_msg);
      IMAGE_RECEIVED = 0;
  }
  ros::spinOnce();
}

  return 0;
}

