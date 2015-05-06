/*
 *      This node is subscribed to 5 different interaction pattern topics:
 *      /last_command, /announce, /gesture, /heading_adj, /dist_adj 
 *      and also to a /presentation topic. This last topic is the one who will
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

class InteractionMonitor {

private:

    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber sub; 
    void last_command_callback (const data_parser::DataParsed& msg);
    void announcement_callback (const data_parser::DataParsed& msg);
    void gesture_callback (const data_parser::DataParsed& msg);
    void heading_adj_callback (const data_parser::DataParsed& msg);
    void dist_adj_callback (const data_parser::DataParsed& msg);
    void category_callback (const data_parser::DataParsed& msg);

    // [publisher attributes]

    /// Variables
    // Annotations vars
    int last_command_store;
    int announcement_store;
    int gesture_store;
    int heading_adj_store;
    int dist_adj_store;
    int category_store;


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
