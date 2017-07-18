/* Capture video input from camera and display on screen */

#include "cv.h"
#include <fstream>
#include "highgui.h"
#include "process.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "motion_analysis_msgs/shapes_msg.h"
#include <boost/algorithm/string/predicate.hpp>
#include "radio_services/InstructionWithAnswer.h"
#include "motion_analysis_msgs/AnswerWithHeader.h"

bool running = false;
ros::NodeHandle *nptr;
ros::Subscriber img_in;
ros::Subscriber conf_in;
std::string conf_topic = "";
std::string image_topic = "";
cv_bridge::CvImagePtr image = 0;
unsigned int IMAGE_RECEIVED = 0;
sensor_msgs::ImageConstPtr ros_image;
motion_analysis_msgs::AnswerWithHeader string_msg;

void configurationCallback(std_msgs::String str){
      if(str.data.compare("h+")==0){
        if(STANDING_PERSON_HEIGHT - 20 >= 0){
          STANDING_PERSON_HEIGHT = STANDING_PERSON_HEIGHT - 20;
	}
	else{
	  STANDING_PERSON_HEIGHT = 0;
	}
      }
      else if(str.data.compare("h-")==0){
        if(STANDING_PERSON_HEIGHT + 20 <= 479){
          STANDING_PERSON_HEIGHT = STANDING_PERSON_HEIGHT + 20;
	}
	else{
	  STANDING_PERSON_HEIGHT = 479;
	}
      }
      else if(str.data.compare("bl-")==0){
        if(OUTOFBED_LEFT - 20 >= 0){
          OUTOFBED_LEFT = OUTOFBED_LEFT - 20;
	}
	else{
	  OUTOFBED_LEFT = 0;
	}
      }
      else if(str.data.compare("bl+")==0){
        if(OUTOFBED_LEFT + 20 <= 639){
          OUTOFBED_LEFT = OUTOFBED_LEFT + 20;
	}
	else{
	  OUTOFBED_LEFT = 639;
	}
      }
      else if(str.data.compare("br-")==0){
        if(OUTOFBED_RIGHT - 20 >= 0){
          OUTOFBED_RIGHT = OUTOFBED_RIGHT - 20;
	}
	else{
	  OUTOFBED_RIGHT = 0;
	}
      }
      else if(str.data.compare("br+")==0){
        if(OUTOFBED_RIGHT + 20 <= 639){
          OUTOFBED_RIGHT = OUTOFBED_RIGHT + 20;
	}
	else{
	  OUTOFBED_RIGHT = 639;
	}
      }
      else if(str.data.compare("cy+")==0){
        if(CUPY - CUPR - 10 >= 0){
          CUPY = CUPY - 10;
	}
	else{
	  CUPY = CUPR;
	}
      }
      else if(str.data.compare("cy-")==0){
        if(CUPY + CUPR + 10 <= 479){
          CUPY = CUPY + 10;
	}
	else{
	  CUPY = 479-CUPR;
	}
      } 
      else if(str.data.compare("cx-")==0){
        if(CUPX - CUPR - 10 >= 0){
          CUPX = CUPX - 10;
	}
	else{
	  CUPX = CUPR;
	}
      }
      else if(str.data.compare("cx+")==0){
        if(CUPX + CUPR + 10 <= 639){
          CUPX = CUPX + 10;
	}
	else{
	  CUPX = 639 - CUPR;
	}
      }
      else if(str.data.compare("cr+")==0){
        if(CUPX - CUPR - 10 >= 0 && CUPY - CUPR - 10 >= 0 && CUPX + CUPR + 10 <= 639 && CUPY + CUPR + 10 <= 479){
          CUPR = CUPR + 10;
        }
      }
      else if(str.data.compare("cr-")==0){
        if(CUPR - 10 >= 10){
          CUPR = CUPR - 10;
        }
        else{
          CUPR = 10;
        }
      }
      else if(boost::starts_with(str.data, "s ")){
        SENSITIVITY = atoi(str.data.substr(2).c_str());
      }
      else if(boost::starts_with(str.data, "ct ")){
        CUPTHRESHOLD = atoi(str.data.substr(3).c_str());
      }
      else if(boost::starts_with(str.data, "cc ")){
        CUPTHRSCOUNT = atoi(str.data.substr(3).c_str());
      }
      else if(str.data.compare("save")==0){
        std::string path = ros::package::getPath("motion_analysis");
        path += "/config/";
        std::string filename = path + "conf.yaml";
        std::ofstream file;
        file.open(filename, std::fstream::out);
        file << "STANDING_PERSON_HEIGHT: " << STANDING_PERSON_HEIGHT << std::endl;
        file << "OUTOFBED_LEFT: " << OUTOFBED_LEFT << std::endl;
        file << "OUTOFBED_RIGHT: " << OUTOFBED_RIGHT << std::endl;
        file << "CUPX: " << CUPX << std::endl;
        file << "CUPY: " << CUPY << std::endl;
        file << "CUPR: " << CUPR << std::endl;
        file << "SENSITIVITY: " << SENSITIVITY << std::endl;
        file << "CUPTHRESHOLD: " << CUPTHRESHOLD << std::endl;
        file << "CUPTHRSCOUNT: " << CUPTHRSCOUNT << std::endl; 
        file.close();
        ROS_WARN("Configuration saved successfully!");
      }
      else if(str.data.compare("reset")==0){
        STANDING_PERSON_HEIGHT = 100;
        OUTOFBED_LEFT = 240;
        OUTOFBED_RIGHT = 460;
        CUPX = 320;
        CUPY = 240;
        CUPR = 40;
        SENSITIVITY = 350;
        CUPTHRESHOLD = 80;
        CUPTHRSCOUNT = 30;
        ROS_WARN("Configuration reset!");
      }
      else{
        ROS_WARN("I got a wrong command!");
      }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& str_img){
  if(IMAGE_RECEIVED == 0){
    ros_image = str_img;
    IMAGE_RECEIVED = 1;
  }
}

/**
 * @brief      This function is called when the corresponding service is called
 *             and based on the parameter passed, either changes its state and
 *             returns the new state, or just returns the current state.
 *             0: Change the current node state to WAITING (false) and return the current node state.
 *             1: Change the current node state to RUNNING (true), return the current node state (and do not change the mode).
 *            10: Change the current node state to RUNNING (true), return the current node state and change the mode to 0.
 *            11: Change the current node state to RUNNING (true), return the current node state and change the mode to 1.
 *            12: Change the current node state to RUNNING (true), return the current node state and change the mode to 2.
 *            -1: Return the current node state.
 *
 * @param[in]  req   The requested action
 * @param[in]  res   The response (the current state of the node)
 *
 * @return     true if everything was successful, false otherwise.
 */
bool nodeStateCallback(radio_services::InstructionWithAnswer::Request &req, radio_services::InstructionWithAnswer::Response &res){

  if(req.command == 0 && running){
    running = false;
    img_in.shutdown();
    conf_in.shutdown();
    ROS_INFO("Stoppped motion_analysis!");
  }
  else if(req.command == 1 || req.command == 10 || req.command == 11 || req.command == 12){
    if(!running){
      img_in = nptr->subscribe<sensor_msgs::Image>(image_topic, 60, imageCallback);
      conf_in = nptr->subscribe<std_msgs::String>(conf_topic, 1, configurationCallback);
      running = true;
    }
    if(req.command >= 10){
      placed = req.command - 10;
    }
    ROS_INFO("Started motion_analysis in mode %d!", placed);
  }

  res.answer = running;
  return true;
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
  nptr = &n;
  bool visualize = false;
  bool configuration_mode = false;
  std::string bounding_box_topic = "";
  std::string motion_analysis_human_topic = "";
  std::string motion_analysis_object_topic = "";
  std::string motion_analysis_shapes_topic = "";
  std::string motion_detection_results_topic = "";
  std::string motion_analysis_node_state_service = "";

  n.param("motion_analysis/visualize", visualize, false);
  n.param("motion_analysis/image_topic", image_topic, std::string("/usb_cam/image_raw"));
  n.param("motion_analysis/bounding_box_topic", bounding_box_topic, std::string("/motion_analysis/bounding_box_viz"));
  n.param("motion_analysis/image_motion_detection_results_topic", motion_detection_results_topic, std::string("/motion_analysis/motion_analysis_viz"));
  n.param("motion_analysis/motion_analysis_shapes_topic", motion_analysis_shapes_topic, std::string("/motion_analysis/shapes_image"));
  n.param("motion_analysis/motion_analysis_human_topic", motion_analysis_human_topic, std::string("/motion_analysis/event/human_transfer"));
  n.param("motion_analysis/motion_analysis_object_topic", motion_analysis_object_topic, std::string("/motion_analysis/event/object_tampered"));
  n.param("motion_analysis/motion_analysis_node_state_service", motion_analysis_node_state_service, std::string("/motion_analysis/node_state"));

  n.param("motion_analysis/STANDING_PERSON_HEIGHT", STANDING_PERSON_HEIGHT, 100);
  n.param("motion_analysis/OUTOFBED_LEFT", OUTOFBED_LEFT, 240);
  n.param("motion_analysis/OUTOFBED_RIGHT", OUTOFBED_RIGHT, 460);
  n.param("motion_analysis/CUPX", CUPX, 320);
  n.param("motion_analysis/CUPY", CUPY, 240);
  n.param("motion_analysis/CUPR", CUPR, 40);
  n.param("motion_analysis/SENSITIVITY", SENSITIVITY, 350);
  n.param("motion_analysis/CUPTHRESHOLD", CUPTHRESHOLD, 80);
  n.param("motion_analysis/CUPTHRSCOUNT", CUPTHRSCOUNT, 30);
  n.param("motion_analysis/configuration_mode", configuration_mode, false);
  n.param("motion_analysis/configuration_keypress_topic", conf_topic, std::string("motion_analysis/configuration/keypress"));

  ros::Publisher image_publisher_bb = n.advertise<sensor_msgs::Image>(bounding_box_topic, 100);
  ros::Publisher image_publisher_mdr = n.advertise<sensor_msgs::Image>(motion_detection_results_topic, 100);
  ros::Publisher shapes_image_publisher = n.advertise<motion_analysis_msgs::shapes_msg>(motion_analysis_shapes_topic, 1);
  ros::Publisher string_publisher_person = n.advertise<motion_analysis_msgs::AnswerWithHeader>(motion_analysis_human_topic, 1);
  ros::Publisher string_publisher_object = n.advertise<motion_analysis_msgs::AnswerWithHeader>(motion_analysis_object_topic, 1);

  ros::ServiceServer service = n.advertiseService(motion_analysis_node_state_service, nodeStateCallback);

  unsigned int iteration, index , showanno;

  showanno = 2;
  iteration = 1;
  int bed_answer;
  int obj_answer;

  motion_analysis_msgs::shapes_msg shapes_image_msg;
  int shapesxy[6] = {};

  ROS_INFO("motion_analysis is now running and it will wait until its state service is called...");

  while(ros::ok()) {
    if(running){
      if(IMAGE_RECEIVED == 1){
        if (iteration > 16383){
            iteration = 0;
          }
          else{
            iteration++;
          }

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
              RGB[639-x][y][REDV  ] = image_data[step*y+(639-x)*3+0];
              RGB[639-x][y][GREENV] = image_data[step*y+(639-x)*3+1];
              RGB[639-x][y][BLUEV ] = image_data[step*y+(639-x)*3+2];
              RGB_unedited[639-x][y][REDV] = unedited_image_data[step*y+(639-x)*3+0];
              RGB_unedited[639-x][y][GREENV] = unedited_image_data[step*y+(639-x)*3+1];
              RGB_unedited[639-x][y][BLUEV] = unedited_image_data[step*y+(639-x)*3+2];
            }
        }

        bed_answer = -1;
        obj_answer = -1;

        process((unsigned char *)RGB, (unsigned char *)Bac, index, showanno, bed_answer, obj_answer, (unsigned char *)RGB_unedited, shapesxy);

        if(!configuration_mode && placed == MODE_HUMAN_MOVEMENT){
          if(bed_answer != -1){
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            string_msg.header = header;
            string_msg.event = bed_answer;
            string_publisher_person.publish(string_msg);
          }
        }
        else if(!configuration_mode && placed == MODE_DETECT_OBJECT_MOVEMENT){
          if(obj_answer != -1){
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            string_msg.header = header;
            string_msg.event = obj_answer;
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
    }
    ros::spinOnce();
  }

  return 0;
}

