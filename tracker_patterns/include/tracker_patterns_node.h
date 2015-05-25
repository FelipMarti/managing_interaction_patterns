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
#include <numeric>

class TrackerPatterns {

private:
    
    ros::NodeHandle n;

    // [subscriber attributes]


    // [publisher attributes]
    ros::Publisher heading_adj_pub;
    ros::Publisher dist_adj_pub;

    // Variables
    std::vector <double> distanceFilter;
    std::vector <double> angleFilter;

    std::vector <double> distancePrevious;
    std::vector <double> anglePrevious;
    
    double currentTimeStamp;

    static const int MAX_DIST_TH = 200;
    static const int MAX_ANGL_TH = 3.14159265358979/4;

    // Functions
    bool check_odom_file(std::ifstream &odom_file, 
                                          const double currentTimeStamp);



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
