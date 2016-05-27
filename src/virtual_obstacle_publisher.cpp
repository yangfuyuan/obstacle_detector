#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "virtual_obstacle_publisher");
  ros::NodeHandle nh;

  ros::Publisher obst_pub = nh.advertise<obstacle_detector::Obstacles>("obstacles", 10);

  obstacle_detector::Obstacles obst_msg;
  obst_msg.header.frame_id = "world";

  obstacle_detector::CircleObstacle circ1, circ2;

  ROS_INFO("Virtual Obstacle Publisher [OK]");

  ros::Time start = ros::Time::now();
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::Time stop = ros::Time::now();
    double t = (stop - start).toSec();

    obst_msg.header.stamp = ros::Time::now();
    obst_msg.circles.clear();

//    // Circular motion example
//    circ1.center.x = 1.5 * cos(0.2 * t) + 0.001 * (rand() % 100);
//    circ1.center.y = 1.5 * sin(0.2 * t) + 0.001 * (rand() % 100);
//    circ1.radius = 0.15 + 0.001 * (rand() % 100);

//    circ2.center.x = 1.5 * cos(-0.2 * t) + 0.001 * (rand() % 100);
//    circ2.center.y = 1.5 * sin(-0.2 * t) + 0.001 * (rand() % 100);
//    circ2.radius = 0.15 + 0.001 * (rand() % 100);

//    // Fusion example
//    if (t < 5.0) {
//      circ1.center.x = -1.20 + 0.2 * t;
//      circ1.center.y = 0.0;
//      circ1.radius = 0.20;

//      circ2.center.x = 1.20 - 0.2 * t;
//      circ2.center.y = 0.0;
//      circ2.radius = 0.20;

//      obst_msg.circles.push_back(circ1);
//      obst_msg.circles.push_back(circ2);
//    }
//    else if (t < 15.0) {
//      circ1.center.x = 0.0;
//      circ1.center.y = 0.0;
//      circ1.radius = 0.20 + 0.20 * exp(-(t - 5.0) / 1.0);

//      obst_msg.circles.push_back(circ1);
//    }

    // Fission example
    if (t < 5.0) {
      circ1.center.x = 0.0;
      circ1.center.y = 0.0;
      circ1.radius = 0.20;

      obst_msg.circles.push_back(circ1);
    }
    else if (t < 6.0) {
      circ1.center.x = 0.0;
      circ1.center.y = 0.0;
      circ1.radius = 0.20 + 0.20 * (1.0 - exp(-(t - 5.0) / 1.0));

      obst_msg.circles.push_back(circ1);
    }
    else if (t < 15.0){
      circ1.center.x = -0.20 - 0.2 * (t - 6.0);
      circ1.center.y = 0.0;
      circ1.radius = 0.20;

      circ2.center.x = 0.20 + 0.2 * (t - 6.0);
      circ2.center.y = 0.0;
      circ2.radius = 0.20;

      obst_msg.circles.push_back(circ1);
      obst_msg.circles.push_back(circ2);
    }

    obst_pub.publish(obst_msg);
    rate.sleep();
  }

  return 0;
}
