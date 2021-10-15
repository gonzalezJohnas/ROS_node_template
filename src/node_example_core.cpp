#include "node_example_core.h"

/*--------------------------------------------------------------------
 * NodeExample()
 * Constructor.
 *------------------------------------------------------------------*/

NodeExample::NodeExample(ros::NodeHandle &n): m_image_buffer(1), m_image_transport(n)
{
    cb = boost::bind(&NodeExample::configCallback, this, _1, _2);
    dr_srv.setCallback(cb);


    // ros parameters
    m_topic_image_input =  std::string("/cam");
    n.getParam("topic_image_input", m_topic_image_input);
    m_topic_image_output =  m_topic_image_input + "/gray";
    n.getParam("topic_image_output", m_topic_image_output);
    m_proc_freq =  1.0;
	n.getParam("max_process_freq", m_proc_freq);
    m_topic_message_output =  std::string("/msg");
	n.getParam("topic_message_output", m_topic_message_output);


    // Publisher for the message
    pub_message = n.advertise<template_node::node_example_data>(m_topic_message_output, 10);
    process_timer_message = n.createTimer(m_proc_freq, &NodeExample::processMessage, this);


    // Subscriber and publisher for the image
	m_sub_image = m_image_transport.subscribe(m_topic_image_input, 10, &NodeExample::imageCallback, this);
	m_pub_image = m_image_transport.advertise(m_topic_image_output, 10);
	process_timer_image = n.createTimer(m_proc_freq, &NodeExample::processImage, this);



} // end NodeExample()

/*--------------------------------------------------------------------
 * ~NodeExample()
 * Destructor.
 *------------------------------------------------------------------*/

NodeExample::~NodeExample()
{
} // end ~NodeExample()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void NodeExample::publishMessage(ros::Publisher *pub_message)
{
  template_node::node_example_data msg;
  msg.message = message;
  msg.threshold = threshold;

  pub_message->publish(msg);
} // end publishMessage()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void NodeExample::messageCallback(const template_node::node_example_data::ConstPtr &msg)
{

  message = msg->message;
  threshold = msg->threshold;

} // end publishCallback()

/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

void NodeExample::configCallback(template_node::template_node_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Dynamic Reconfigure Triggered");
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message = config.message.c_str();
  threshold = config.threshold;


} // end configCallback()

/*--------------------------------------------------------------------
 * imageCallback()
 * Callback function called when a new image has arrived on 
 * the "camera/image" topic.
 *------------------------------------------------------------------*/

void NodeExample::imageCallback(const sensor_msgs::ImageConstPtr& msg)

{
    // Lock the ressource and write
    lock_t scope_lock(m_image_buffer_mutex);
	m_image_buffer.push_back(msg);

}


/*--------------------------------------------------------------------
 * processImage()
 * Function called prior to publish the image 
 *------------------------------------------------------------------*/

  void NodeExample::processImage(const ros::TimerEvent& event){
      try
	{
		lock_t scope_lock(m_image_buffer_mutex);
		if(m_image_buffer.size() <= 0)
		{
			return;
		}
		sensor_msgs::ImageConstPtr msg = m_image_buffer[0];
		m_image_buffer.pop_front();
		scope_lock.unlock();

        cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

		cv::Mat image_in, image_out;

        if (frame.channels() > 1)cv::cvtColor(frame, image_in, cv::COLOR_BGR2GRAY);

        cv::threshold(image_in, image_out, this->threshold, 255, cv::THRESH_TOZERO);



        if(m_pub_image.getNumSubscribers() > 0)
		{
            
			sensor_msgs::Image::Ptr output_msg = cv_bridge::CvImage(msg->header, "mono8", image_out).toImageMsg();
			m_pub_image.publish(output_msg);
		}

    }
	catch (cv::Exception &e)
	{
		ROS_WARN("Error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
	}
  }


/*--------------------------------------------------------------------
 * processMessage()
 * Function called prior to publish the message 
 *------------------------------------------------------------------*/

  void NodeExample::processMessage(const ros::TimerEvent& event){
    template_node::node_example_data msg;
    msg.message = message;
    msg.threshold = threshold;

   
    pub_message.publish(msg);
      
  }