/*
 *      This node is subscribed to 5 different interaction pattern topics:
 *      /last_command, /announce, /gesture, /heading_adj, /dist_adj 
 *      and also to the /category topic. This last topic is the one who will
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





}

InteractionMonitor::~InteractionMonitor (void)
{
}


void InteractionMonitor::last_command_callback (const data_parser::DataParsed& msg)
{
}


void InteractionMonitor::announcement_callback (const data_parser::DataParsed& msg)
{
}

void InteractionMonitor::gesture_callback (const data_parser::DataParsed& msg)
{
}

void InteractionMonitor::heading_adj_callback (const data_parser::DataParsed& msg)
{
}

void InteractionMonitor::dist_adj_callback (const data_parser::DataParsed& msg)
{
}

void InteractionMonitor::category_callback (const data_parser::DataParsed& msg)
{
}

int InteractionMonitor::Main () {



}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "interaction_monitor");
    
    InteractionMonitor foo;
    return foo.Main();

}

