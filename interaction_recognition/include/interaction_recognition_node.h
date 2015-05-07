/*
 *      This node is used to perform inference in a Bayesian network.
 *      It's subscribed to interaction_monitor /bn_vars topic 
 *      to obtain data and then performs inference with a BN using SMILE libraries
 *
 *      At the end it prints some statistics with the performance.
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT)
 *      Copyright (c) 2015 Felip Marti Carrillo
 */


#ifndef _INTERACTION_RECOGNITION_NODE_HPP
#define _INTERACTION_RECOGNITION_NODE_HPP

#include "ros/ros.h"
#include "interaction_monitor/BayesianNetworkVariable.h"
#include "smile.h"

class InteractionRecognition {

private:

    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber sub; 
    void perform_inference_callback (const interaction_monitor::BayesianNetworkVariable& msg);

    /// Variables
    // Bayesian Network
    DSL_network theNet;

    // Statistics
    int Stats_Results[9]; 
    // Stats_Results[0] => Correct; 
    // Stats_Results[1] => Epic Failure; 
    // Stats_Results[2] => 2 Similar; 
    // Stats_Results[3] => 3 Similar; 
    // Stats_Results[4] => 4 Similar; 
    // Stats_Results[5] => Unknown has category; 
    // Stats_Results[6] => 2 Similar but not the category, so FAIL; 
    // Stats_Results[7] => 3 Similar but not the category, so FAIL; 
    // Stats_Results[8] => WTF!; 
    std::vector<std::string> statsFails;
    std::vector<std::string> statsBetween2;
    std::vector<std::string> statsAmong3;
    // Max difference to consider 2 objects different
    double MAX_DIFF;

    void print_statistics();
    
    


public:

    /**
     *  InteractionLearner Constructor 
     */
    InteractionRecognition (void);

    /**
     *  InteractionLearner Destructor 
     */
    ~InteractionRecognition (void);

    int Main (const char* path);


};


#endif



