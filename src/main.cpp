/**
 * Distributed under the MPL 2.0 License (license terms found in LICENSE or at https://opensource.org/licenses/MPL-2.0)
 * @file main.cpp
 * @brief Script to demo robot collector
 * @author Pablo Sanhueza
 * @author Ryan Cunningham
 * @author Andre Gomes
 * @copyright 2019 Pablo Sanhueza, Ryan Cunningham, Andre Gomes
 */


/**
 * @brief main function
 * @param argc  The argc
 * @param argv  The argv
 * @return none
 */
int main(int argc, char **argv) {
  /// Initializing the node
  ros::init(argc, argv, "temp");

  /// Publishes at 5 Hz
  ros::Rate loop_rate(5);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
