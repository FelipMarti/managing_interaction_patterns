/**
 *      This node calls the ROS-service parser for each input file 
 *      and publishes all the Annotations that appear in time intervals of 500ms
 *      
 *      Before publishing those annotations, they are translated from strings to integers
 *      in order to transform from ELAN variables to Bayesian network variables.
 *
 *      The trigger topic will be published with a "true" when the end time of the usr_present 
 *      annotation is finished. This trigger will be used to train or perform inference 
 *      with all the immediate annotations in the subscriber nodes.
 *
 *      When there is no more data and before closing the node the trigger topic 
 *      will publish "false"
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
#include "data_parser/Parse.h"
#include "data_parser/DataParsed.h"
#include "interaction_monitor/AnnotationVariable.h"
#include "std_msgs/Bool.h"

class ElanTranslator {

private:
    
    ros::NodeHandle n;

    // [subscriber attributes]


    // [publisher attributes]
    ros::Publisher last_command_pub;
    ros::Publisher announce_pub;
    ros::Publisher gesture_pub;
    ros::Publisher heading_adj_pub;
    ros::Publisher dist_adj_pub;
    ros::Publisher category_pub;
    ros::Publisher trigger_pub;

    // Dictionaries
    std::map<std::string,int> ObjCategory;
    void write_category_dictionary(void);
    std::map<std::string,int> GestureCategory;
    void write_gesture_dictionary(void);
    std::map<std::string,int> CommandCategory;
    void write_command_dictionary(void);

    // Var to store annotations
    std::vector <data_parser::DataParsed> dataXmlFile;


public:

    /**
     *  ELanTranslator Constructor 
     */
    ElanTranslator (void);

    /**
     *  ElanTranslator Destructor 
     */
    ~ElanTranslator (void);

    int Main (int argc, char **argv);
               

};


#endif
