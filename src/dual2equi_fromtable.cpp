/* 2020/04/06 - G. CARON
   ROS node for dual fisheye to equirectangular mapping
   listens a dual fisheye image on topic /...
   publishes athe tranformed equirectangular image on topic /...
*/

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <fstream>

//#define VERBOSE
//#define MEASUREMAPPINGTIME

// global variables
image_transport::Publisher pub_equirectangular_image;
sensor_msgs::ImagePtr EquiImage_msg;
cv::Mat EquiImage;
cv_bridge::CvImagePtr pt_DualImage;

int imWidth, imHeight;
unsigned int nbPixels;
int *coordMapping[4] = {NULL, NULL, NULL, NULL}; // equirectangular and dual fisheye image pixel coordinates

// functions prototypes
void init(std::string & coordinatesTableFilename);
void imageCallback(const sensor_msgs::ImageConstPtr& Image);

int main (int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "dual2equi_fromtable");
  ros::NodeHandle nH;

#ifdef VERBOSE
  ROS_INFO("dual2equi_fromtable::main");
#endif

  // ROS read launch file parameters
  // coordinates table filename

  ros::NodeHandle nHp("~"); //to get the parameters for that node
  std::string inputImagesTopic, outputImagesTopic, coordinatesTableFilename;
  nHp.param("inputImagesTopic", inputImagesTopic, std::string(""));
  nHp.param("outputImagesTopic", outputImagesTopic, std::string(""));
  nHp.param("coordinatesTableFilename", coordinatesTableFilename, std::string(""));

  nHp.param("imWidth", imWidth, 0);
  nHp.param("imHeight", imHeight, 0);

  nHp.getParam("inputImagesTopic", inputImagesTopic);
  nHp.getParam("outputImagesTopic", outputImagesTopic);
  nHp.getParam("coordinatesTableFilename", coordinatesTableFilename);

  nHp.getParam("imWidth", imWidth);
  nHp.getParam("imHeight", imHeight);

  nbPixels = imWidth*imHeight;

  init(coordinatesTableFilename);

  // ROS Listner
  image_transport::ImageTransport i_t(nH);
  image_transport::Subscriber sub_image = i_t.subscribe(inputImagesTopic, 1, imageCallback);

  // ROS Publisher
  pub_equirectangular_image = i_t.advertise(outputImagesTopic, 1);

  ros::spin();

  //after killing ros node
  for(unsigned int c = 0 ; c < 4 ; c++)
    if(coordMapping[c] != NULL)
    {
      delete [] coordMapping[c];
      coordMapping[c] = NULL;
    }

  return 0;
}

void init(std::string & coordinatesTableFilename)
{
  if(nbPixels == 0)
  {
    ROS_INFO("dual2equi_fromtable::init: nbPixels == 0");
    return;
  }

  // Get pixel coordinates mapping
  for(unsigned int c = 0 ; c < 4 ; c++)
    coordMapping[c] = new int[nbPixels];

  std::ifstream ficCoords(coordinatesTableFilename.c_str());

  unsigned int pix = 0;
  while(!ficCoords.eof())
  {
    ficCoords >> coordMapping[0][pix] >> coordMapping[1][pix] >> coordMapping[2][pix] >> coordMapping[3][pix];
    pix++;
  }

  // Initialize the image to be published
  EquiImage.create(imHeight, imWidth, CV_8UC3);
}

void imageCallback(const sensor_msgs::ImageConstPtr& DualImage_msg)
{
#ifdef VERBOSE
  ROS_INFO("dual2equi_fromtable::imageCallback");
#endif

//  pt_DualImage = cv_bridge::toCvShare(DualImage_msg, sensor_msgs::image_encodings::BGR8); // not compiling
  pt_DualImage = cv_bridge::toCvCopy(DualImage_msg, sensor_msgs::image_encodings::BGR8);

#ifdef MEASUREMAPPINGTIME
  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();
#endif

  int *pt_ue = coordMapping[0], *pt_ve = coordMapping[1], *pt_ud = coordMapping[2], *pt_vd = coordMapping[3];

  //Easy to read but unoptimized version
/*  for(unsigned int p = 0; p < nbPixels ; p++, pt_ue++, pt_ve++, pt_ud++, pt_vd++)
  {
    EquiImage.at<cv::Vec3b>(*pt_ve, *pt_ue) = pt_DualImage->image.at<cv::Vec3b>(*pt_vd, *pt_ud);
  }
*/

/*
  //Optimized version (proc time / 2)
  unsigned int pix = 0;
  for(unsigned int p = 0; p < nbPixels ; p++, pt_ue++, pt_ve++, pt_ud++, pt_vd++)
  {
    memcpy(EquiImage.data + 3*((*pt_ve)*imWidth+(*pt_ue)), pt_DualImage->image.data + 3*((*pt_vd)*imWidth+(*pt_ud)), 3);
  }
*/

  //Optimized version with specific knowledge on the equirect coordinates (a few ms less)
  unsigned char *pt_equi = EquiImage.data;// = EquiImage_msg->data;

  unsigned int pix = 0;
  for(unsigned int p = 0; p < nbPixels ; p++, pt_equi+=3, pt_ud++, pt_vd++)
  {
    memcpy(pt_equi, pt_DualImage->image.data + 3*((*pt_vd)*imWidth+(*pt_ud)), 3);
  }

#ifdef MEASUREMAPPINGTIME
  end_ = ros::WallTime::now();

  double execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
#endif

  EquiImage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", EquiImage).toImageMsg();
  pub_equirectangular_image.publish(EquiImage_msg);
}

