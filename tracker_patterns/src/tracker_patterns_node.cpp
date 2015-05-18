/**
 *      
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT) 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */

#include "tracker_patterns_node.h"


TrackerPatterns::TrackerPatterns (void)
{

    /// Init publishers
    heading_adj_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_heading_adj",10);
    dist_adj_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_dist_adj",10);

}


TrackerPatterns::~TrackerPatterns (void)
{
}

int TrackerPatterns::Main (int argc, char **argv) 
{

    ros::Rate loop_rate(2000);


    /// Extracting data 
    std::ifstream odom_file;
    std::ifstream traj_file;


    for (int i=0;i<argc-2;i+=2) {
//TODO: this is not working well, check it!!!!!
        char str[100];
        strcpy (str,argv[1]);
        strcat (str,argv[i+2]);
        odom_file.open( str ); 
        if ( odom_file.is_open() ) {
            ROS_INFO("[tracker_patterns] %s",str);
        }
        else {
            ROS_INFO("[tracker_patterns] NOT OPEN");
        }
        strcpy (str,argv[1]);
        strcat (str,argv[i+3]);
        traj_file.open( str );
        if (odom_file.is_open() ) {
            ROS_INFO("[tracker_patterns] %s",str);
        }
        else {
            ROS_INFO("[tracker_patterns] NOT OPEN");
        }





    }

    ROS_INFO("[tracker_patterns] Waiting 0.5s to get ready the publisher");
    ros::Duration(0.5).sleep();
    ROS_INFO("[tracker_patterns] Start Publishing!");

    /// Main Loop
    while (ros::ok()) {
    
        ros::spinOnce();
        loop_rate.sleep();

    }


    ROS_WARN("[tracker_patterns] No more data!");
    ROS_WARN("[tracker_patterns] So, the node will be stopped gently :)");

    odom_file.close();
    traj_file.close();

    exit(0);

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracker_patterns_node");
    if (argc < 4) {
        ROS_ERROR("[tracker_patterns] usage: tracker_patterns_node" 
                  "PATH_FILE(s) ODOM_FILE_1 TRAJ_FILE_1 .. ODOM_FILE_N TRAJ_FILE_N");
        exit(1);
    }

    TrackerPatterns foo;
    return foo.Main(argc, argv);

}




