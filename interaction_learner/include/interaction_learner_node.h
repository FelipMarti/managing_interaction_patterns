/*
 *      This node is used to train a Bayesian network.
 *      It's subscribed to interaction_monitor /BN_vars topic
 *      to obtain data and then generates a BN using SMILEarn library
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT)
 *      Copyright (c) 2015 Felip Marti Carrillo
 */


#ifndef _INTERACTION_LEARNER_NODE_HPP
#define _INTERACTION_LEARNER_NODE_HPP

#include "ros/ros.h"
#include "interaction_monitor/BayesianNetworkVariable.h"
#include "smile.h"
#include "smilearn.h"

class InteractionLearner {

private:
    
    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber data_sub;
    void read_data_callback(const interaction_monitor::BayesianNetworkVariable& msg); 

    // [publisher attributes]
    ros::Publisher joint_pub;


    /// Variables
    // Bayesian Network
    DSL_dataset dataBayesianNetwork;
    void set_nomenclature_to_dataset(void);
    std::string pathBN;
    DSL_network net;
    
    // Vectors to store data to train the BN 
    std::vector<int> lastCommandData;
    std::vector<int> usrAnnounceData;
    std::vector<int> usrGestureData;
    std::vector<int> headingAdjData;
    std::vector<int> distanceAdjData;
    std::vector<int> category;

    // Subscriber counter var
    int subCounter;


public:

    /**
     *  InteractionLearner Constructor 
     */
    InteractionLearner (void);

    /**
     *  InteractionLearner Destructor 
     */
    ~InteractionLearner (void);
    
    int Main (const char* path, const char* file);




};


#endif
