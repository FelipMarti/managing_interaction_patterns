/**
 *      This node calls the parser ROS-service, once for each input file 
 *      and publishes all the Annotations that appears in time intervals of 500ms
 *      
 *      Before publishing those annotations are translated from strings to integers
 *      in order to train or perform inference later in a Bayesian network
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
