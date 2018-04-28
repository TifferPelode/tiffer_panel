///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/20/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#ifndef STATUS_LOGGER_H
#define STATUS_LOGGER_H

#include <string>
#include <map>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * A helper C++ class for publishing messages designed for the StatusPanel RViz plugin. Handles HTML
 * colors and deals with the ROS setup/teardown.
 */
class StatusLogger
{
public:
  /**
   * Create a new StatusLogger that will publish to the supplied topic. The default text color is black.
   * @param topic the ROS topic to publish to. This can be changed in the future using setTopic().
   * @param defaultColor by default, all status messages posted to the StatusPanel will have this color.
   *                     this can be overridden on a message-by-message basis, or can be changed later
   *                     using setDefaultColor().
   * See the documentation for setDefaultColor() for more information on possible color codes.
   */
  StatusLogger(std::string topic = "/status", std::string defaultColor = "");

  /**
   * @brief Set the default color to use for messages.
   * The supplied color will be inserted into CSS styling for the posted messages. In general, you should usually use one of two types of color definition:
   *   - Hex values, such as #000 (black) or #a9144c (crimson). See 
   *   - Named colors as defined by CSS. See http://www.w3schools.com/cssref/css_colors_group.asp.
   * This is not a complete list. Any CSS-value color will work. See http://www.w3schools.com/cssref/css_colors_legal.asp for all options.
   *
   * If the color is not valid, the default will be the RViz/Qt default (most likely black).
   *
   * @param defaultColor the new default color to set.
   */ 
  void setDefaultColor(const std::string& defaultColor);

  /**
   * Set the topic to publish status messages to.
   * This will result in a publisher restart if the name changes.
   * @param topicName the topic to publish status messages to.
   */
  void setTopic(std::string topicName);

  /**
   * Logs a status message that StatusPanel can understand, in the given color.
   * This is a fancy alias for logHTML() that allows you to set the text color on-the-fly.
   * @param message the message text to publish.
   * @param color an optional color override string. See documentation for setDefaultColor() for a list of possible color values.
   */
  void log(std::string message, std::string color);

  /**
   * logs a status message that StatusPanel can understand. This posts raw HTML as a String message.
   * @param htmlMessage the message to publish. It will not be modified in this function, just passed on.
   */
  void logHTML(std::string htmlMessage);

  ///Message queue size for publisher.
  const static unsigned int MESSAGE_QUEUE_SIZE = 100;

protected:
  ///The topic to post messages to
  std::string topic;

  ///The default color string to use when formatting.
  std::string defaultColor;

  /// The ROS publisher for the incoming messages.
  ros::Publisher publisher;

  /// The ROS node handle.
  ros::NodeHandle nh;

  /// The message to use for publishing.
  std_msgs::String msg;
};

#endif

