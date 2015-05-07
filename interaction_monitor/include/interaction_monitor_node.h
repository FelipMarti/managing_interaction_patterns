/*
 *      This node is subscribed to 5 different interaction pattern topics:
 *      /last_command, /announce, /gesture, /heading_adj, /dist_adj 
 *      also to the /category and /trigger topic. This last topic is the one who will
 *      trigger the publisher.
 *      
 *      The publisher is a vector of Bayesian Network variables that will train
 *      or perform inference in another node with a BN.
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT) 
 *      Copyright (c) 2015 Felip Marti Carrillo 
 */

#ifndef _INTERACTION_MONITOR_NODE_HPP
#define _INTERACTION_MONITOR_NODE_HPP

#include "ros/ros.h"
#include "data_parser/DataParsed.h"
#include "interaction_monitor/AnnotationVariable.h"
#include "interaction_monitor/BayesianNetworkVariable.h"
#include "std_msgs/Bool.h"

class InteractionMonitor {

private:

    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber last_command_sub; 
    ros::Subscriber announcement_sub; 
    ros::Subscriber gesture_sub; 
    ros::Subscriber heading_adj_sub; 
    ros::Subscriber dist_adj_sub; 
    ros::Subscriber category_sub; 
    ros::Subscriber trigger_sub; 

    void last_command_callback (const interaction_monitor::AnnotationVariable& msg);
    void announcement_callback (const interaction_monitor::AnnotationVariable& msg);
    void gesture_callback (const interaction_monitor::AnnotationVariable& msg);
    void heading_adj_callback (const interaction_monitor::AnnotationVariable& msg);
    void dist_adj_callback (const interaction_monitor::AnnotationVariable& msg);
    void category_callback (const interaction_monitor::AnnotationVariable& msg);
    void trigger_callback (const std_msgs::Bool& msg);

    // [publisher attributes]
    ros::Publisher bnVars_pub;


    /// Variables
    // Annotations vars
    interaction_monitor::AnnotationVariable lastCommandStore;
    interaction_monitor::AnnotationVariable announcementStore;
    interaction_monitor::AnnotationVariable gestureStore;
    interaction_monitor::AnnotationVariable headingAdjStore;
    interaction_monitor::AnnotationVariable distAdjStore;
    interaction_monitor::AnnotationVariable categoryStore;
    
    // Publisher counter var
    int pubCounter;


public:

    /** 
     *  InteractionMonitor Constructor 
     */
    InteractionMonitor (void);

    /** 
     *  InteractionMonitor Destructor 
     */
    ~InteractionMonitor (void);

    int Main ();


};


#endif
