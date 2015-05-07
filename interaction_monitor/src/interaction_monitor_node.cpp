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


#include "interaction_monitor_node.h"

InteractionMonitor::InteractionMonitor (void)
{

    // Init Subscribers
    last_command_sub = this->n.subscribe("last_command", 10,
                                &InteractionMonitor::last_command_callback, this);
    announcement_sub = this->n.subscribe("announce", 10,
                                &InteractionMonitor::announcement_callback, this);
    gesture_sub = this->n.subscribe("gesture", 10,
                                &InteractionMonitor::gesture_callback, this);
    heading_adj_sub = this->n.subscribe("heading_adj", 10,
                                &InteractionMonitor::heading_adj_callback, this);
    dist_adj_sub = this->n.subscribe("dist_adj", 10,
                                &InteractionMonitor::dist_adj_callback, this);
    category_sub = this->n.subscribe("category", 10,
                                &InteractionMonitor::category_callback, this);
    trigger_sub = this->n.subscribe("trigger", 1,
                                &InteractionMonitor::trigger_callback, this);

    // Init Publishers
    bnVars_pub = n.advertise<interaction_monitor::BayesianNetworkVariable>("BN_vars",1);

    pubCounter = 0;

}


InteractionMonitor::~InteractionMonitor (void)
{
}


/** 
 *  5 subscribers with interactive patterns annotations
 */
void InteractionMonitor::last_command_callback 
                         (const interaction_monitor::AnnotationVariable& msg)
{
    lastCommandStore = msg;
}


void InteractionMonitor::announcement_callback 
                         (const interaction_monitor::AnnotationVariable& msg)
{
    announcementStore = msg;
}


void InteractionMonitor::gesture_callback 
                         (const interaction_monitor::AnnotationVariable& msg)
{
    gestureStore = msg;
}


void InteractionMonitor::heading_adj_callback 
                         (const interaction_monitor::AnnotationVariable& msg)
{
    headingAdjStore = msg;
}


void InteractionMonitor::dist_adj_callback 
                         (const interaction_monitor::AnnotationVariable& msg)
{
    distAdjStore = msg;
}


/** 
 *  Subscriber for the object category 
 */
void InteractionMonitor::category_callback 
                         (const interaction_monitor::AnnotationVariable& msg)
{
    categoryStore = msg;
}


/** 
 *  Subscriber used to publish all the data. 
 */
void InteractionMonitor::trigger_callback (const std_msgs::Bool& msg)
{
    // Checking if the bool received is true to publish annotations.
    // if its false we will publish -1s
    if (msg.data) {
                
        /// Filling publisher var with annotations taking into account the time 10s
        interaction_monitor::BayesianNetworkVariable tmpPubVar;
        const int tenSeconds = 10000;

        // Last command
        if (categoryStore.tini-tenSeconds <= lastCommandStore.tend and 
            categoryStore.tend >= lastCommandStore.tini) {
            
            tmpPubVar.last_cmd = lastCommandStore.value;
        }
        else {
            tmpPubVar.last_cmd = 5; // none
        }

        // Announcement
        if (categoryStore.tini-tenSeconds <= announcementStore.tend and 
            categoryStore.tend >= announcementStore.tini) {
            
            tmpPubVar.announce = announcementStore.value;
        }
        else {
            tmpPubVar.announce = 0; // no
        }

        // Gesture
        if (categoryStore.tini-tenSeconds <= gestureStore.tend and 
            categoryStore.tend >= gestureStore.tini) {
            
            tmpPubVar.gesture = gestureStore.value;
        }
        else {
            tmpPubVar.gesture = 5; // none
        }

        // Heading adjustment 
        if (categoryStore.tini-tenSeconds <= headingAdjStore.tend and 
            categoryStore.tend >= headingAdjStore.tini) {
            
            tmpPubVar.head_adj = headingAdjStore.value;
        }
        else {
            tmpPubVar.head_adj = 0; // no
        }

        // Distance adjustment 
        if (categoryStore.tini-tenSeconds <= distAdjStore.tend and 
            categoryStore.tend >= distAdjStore.tini) {
            
            tmpPubVar.dist_adj = distAdjStore.value;
        }
        else {
            tmpPubVar.dist_adj = 0; // no
        }

        // Category 
        if (categoryStore.tini-tenSeconds <= categoryStore.tend and 
            categoryStore.tend >= categoryStore.tini) {
            
            tmpPubVar.category = categoryStore.value;
        }
        else {
            tmpPubVar.category = 3; // unknown  
        }

        /// Publishing
        pubCounter++;
        ROS_INFO("[interaction_monitor] Triggered! Publication #%d",pubCounter);
        bnVars_pub.publish(tmpPubVar);

    }
    else {

        // -1 when there are no more annotations coming
        interaction_monitor::BayesianNetworkVariable tmpPubVar;
        tmpPubVar.category=-1;
        bnVars_pub.publish(tmpPubVar);

    }
}


int InteractionMonitor::Main (void) 
{
     ros::spin();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "interaction_monitor");
    
    InteractionMonitor foo;
    return foo.Main();
}

