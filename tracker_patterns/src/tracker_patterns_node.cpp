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

    ROS_INFO("[tracker_patterns] Waiting 0.5s to get ready the publisher");
    ros::Duration(0.5).sleep();
    ROS_INFO("[tracker_patterns] Start Publishing!");


    // For all the inputs
    for (int i=0;i<argc-2;i+=2) {

        /**
         *  Opening and extracting all data for each file entered as a parameter. 
         */
        std::ifstream odom_file;
        std::ifstream traj_file;

        char str[100];
        // Concatenating absolute path with file
        strcpy (str,argv[1]);
        strcat (str,"odom/");
        strcat (str,argv[i+2]);
        odom_file.open( str ); 
        if ( odom_file.is_open() ) {
            ROS_INFO("[tracker_patterns] Opening: %s",str);
        }
        else {
            ROS_ERROR("[tracker_patterns] NOT OPEN: %s",str);
            ROS_ERROR("[tracker_patterns] ERROR, closing node");
            exit(1);
        }
        // Concatenating absolute path with file
        strcpy (str,argv[1]);
        strcat (str,"traj/");
        strcat (str,argv[i+3]);
        traj_file.open( str );
        if (traj_file.is_open() ) {
            ROS_INFO("[tracker_patterns] Opening: %s",str);
        }
        else {
            ROS_ERROR("[tracker_patterns] NOT OPEN: %s",str);
            ROS_ERROR("[tracker_patterns] ERROR, closing node");
            exit(1);
        }

        bool endFile = false;
        // This var currentTimeStamp will be updated by a subscriber in a future
        currentTimeStamp = 1274451145;
        double timeStamp = currentTimeStamp;   //TimeStamp read from the file

        /**
         *  Main Loop. Looping and publishing until the end of the file.
         */
        while (ros::ok() and !endFile) {
            
            // Initialisations
            std::string line;
            double dummy, distance, angle;
            distanceFilter.resize(0); 
            angleFilter.resize(0); 
            int len;   //Var to store position of the line in a file

            /**
             *  The main idea in this loop is to calc the average distance 
             *  value and angle that happens in interval times of 0.5s
             *  For instance:
             *                         1274451145.0 - 1274451145.5
             *                         1274451145.5 - 1274451146.0
             */
            const double FIVEMS = 0.5;
            while ( !endFile and currentTimeStamp+FIVEMS > timeStamp ) {

                // Get current position in buffer file
                len = traj_file.tellg();
                // Read a line
                endFile = std::getline(traj_file, line).eof();

                // Reading columns
                std::istringstream ss(line);
                ss >> timeStamp >> dummy >> dummy >> dummy >> dummy >> dummy 
                >> dummy >> distance >> angle;

                // Storing values to calculate average value
                distanceFilter.push_back(distance);
                angleFilter.push_back(angle);

            }

            /**
             *  The last value stored belongs to the next time interval.
             *  because the time stamp value is updated after processing 
             *  distance and angle values.
             *  Therefore, here we remove that value from the vector
             *  and return that line to the buffer.
             */
            if ( distanceFilter.size() != 0) {
                distanceFilter.pop_back();
                angleFilter.pop_back();
                // Return to position before "Read line".
                traj_file.seekg(len ,std::ios_base::beg);
            }

            /**
             *  Filtering, updating last values and checking thresholds
             *  considering 0.5s and 1s before. 
             *  
             *  If there are no previous values to compare, it doesn't publish 
             */
            bool distanceAdj=false;
            bool angleAdj=false;
            if (distanceFilter.size() != 0) {

                // Filtering values 
                double sumElemsD =std::accumulate(distanceFilter.begin(),distanceFilter.end(),0);
                distancePrevious.push_back(sumElemsD/distanceFilter.size());
                double sumElemsA =std::accumulate(angleFilter.begin(),angleFilter.end(),0);
                anglePrevious.push_back(sumElemsA/angleFilter.size());

                // Storing value
                distancePrevious.push_back(sumElemsD/distanceFilter.size());
                anglePrevious.push_back(sumElemsA/angleFilter.size());

                // Checking DISTANCE threshold
                if (distancePrevious.size() > 3) {
                    distancePrevious.erase (distancePrevious.begin()); 
                    // THRESHOLD
                    if ( abs (distancePrevious[2] - distancePrevious[1]) > MAX_DIST_TH or
                         abs (distancePrevious[2] - distancePrevious[0]) > MAX_DIST_TH) {
                        distanceAdj=true;
                    }
                }
                // Checking ANGLE threshold
                if (anglePrevious.size() > 3) {
                    anglePrevious.erase (anglePrevious.begin()); 
                    // THRESHOLD
                    // TODO: CHECK THRESHOLD
                    if ( abs (anglePrevious[2] - anglePrevious[1]) > MAX_ANGL_TH or
                         abs (anglePrevious[2] - anglePrevious[0]) > MAX_ANGL_TH) {
                        angleAdj=true;
                    }
                }

            }

            /**
             *  Checking in odometry file if the distance or angle adjustment is
             *  because of the robot movement
             */
            bool odomValues = false;
            if (distanceAdj or angleAdj) {
                //TODO: CHECK odom  
                odomValues = check_odom_file(odom_file, currentTimeStamp);
                
            }
            
            //TODO: Publish 

            // This var will be updated by a subscriber in a future
            currentTimeStamp+=0.5;
    
            ros::spinOnce();
            loop_rate.sleep();

        }

        odom_file.close();
        traj_file.close();

    }

    ROS_WARN("[tracker_patterns] No more data!");
    ROS_WARN("[tracker_patterns] So, the node will be stopped gently :)");

    exit(0);

}


bool TrackerPatterns::check_odom_file(std::ifstream &odom_file,
                                        const double currentTimeStamp) 
{

//TODO: make it work...
//TODO: Check values, currentTimeStamp, etc...!
std::cout<< "[check_odom_file] " << currentTimeStamp << std::fixed << std::endl;
    std::string line;
    int currentTimeStampTrunc = currentTimeStamp;
    for(unsigned int curLine = 0; getline(odom_file, line); curLine++) {
std::cout<<line<<"  "<<currentTimeStamp<<std::endl;
        if (line.find( currentTimeStampTrunc ) != std::string::npos) {
            std::cout << "found: " << currentTimeStampTrunc << "line: " << curLine << std::endl;
        }
    }
    
    
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




