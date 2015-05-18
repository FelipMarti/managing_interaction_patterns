/**
 *      
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT) 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */

#ifndef _ELAN_TRANSLATOR_NODE_HPP
#define _ELAN_TRANSLATOR_NODE_HPP

#include "ros/ros.h"
#include "interaction_monitor/AnnotationVariable.h"
#include <fstream>

class TrackerPatterns {

private:
    
    ros::NodeHandle n;

    // [subscriber attributes]


    // [publisher attributes]
    ros::Publisher heading_adj_pub;
    ros::Publisher dist_adj_pub;



public:

    /**
     *  ELanTranslator Constructor 
     */
    TrackerPatterns (void);

    /**
     *  TrackerPatterns Destructor 
     */
    ~TrackerPatterns (void);

    int Main (int argc, char **argv);
               

};


#endif
