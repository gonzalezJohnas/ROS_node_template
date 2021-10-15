#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "template_node/node_example_data.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include "template_node_paramsConfig.h"

// Include to get image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <mutex>

using std::string;

class NodeExample
{
public:
    //! Constructor.
    NodeExample(ros::NodeHandle &n);

    //! Destructor.
    ~NodeExample();

    //! Callback function for dynamic reconfigure server.
    void configCallback(template_node::template_node_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage(ros::Publisher *pub_message);

    //! Callback function for subscriber.
    void messageCallback(const template_node::node_example_data::ConstPtr &msg);

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void processImage(const ros::TimerEvent &event);

    void processMessage(const ros::TimerEvent &event);

    // Dynamic reconfigure parameters
    dynamic_reconfigure::Server<template_node::template_node_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<template_node::template_node_paramsConfig>::CallbackType cb;

private:
    //! The actual message.
    string message;

    //! The first integer to use in addition.
    int threshold;

    // Message publisher
    ros::Publisher pub_message;
    std::string m_topic_message_output;

    // Image parameters
    image_transport::ImageTransport m_image_transport;
    image_transport::Publisher m_pub_image;
    image_transport::Subscriber m_sub_image;

    std::string m_topic_image_input;
    std::string m_topic_image_output;

    typedef boost::mutex::scoped_lock lock_t;
    boost::mutex m_image_buffer_mutex;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> m_image_buffer;

    // Process timer
    double m_proc_freq;
    ros::Timer process_timer_image;
    ros::Timer process_timer_message;
};

#endif // SR_NODE_EXAMPLE_CORE_H
