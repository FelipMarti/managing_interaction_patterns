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
        char odometryPathFile[100];
        // Concatenating absolute path with odometry file
        strcpy (odometryPathFile,argv[1]);
        strcat (odometryPathFile,"odom/");
        strcat (odometryPathFile,argv[i+2]);

        std::ifstream traj_file;
        char trajectoryPathFile[100];
        // Concatenating absolute path with trajectory file name, and opening
        strcpy (trajectoryPathFile,argv[1]);
        strcat (trajectoryPathFile,"traj/");
        strcat (trajectoryPathFile,argv[i+3]);
        traj_file.open( trajectoryPathFile );
        if (traj_file.is_open() ) {
            ROS_INFO("[tracker_patterns] Opening trajectory file: %s",trajectoryPathFile);
        }
        else {
            ROS_ERROR("[tracker_patterns] NOT OPEN: %s",trajectoryPathFile);
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
                //TODO: CHECK value returned  
                odomValues = check_odom_file(odometryPathFile, currentTimeStamp);
                
            }
            
            //TODO: Publish 

            // This var will be updated by a subscriber in a future
            currentTimeStamp+=0.5;
    
            ros::spinOnce();
            loop_rate.sleep();

        }

        traj_file.close();

    }

    ROS_WARN("[tracker_patterns] No more data!");
    ROS_WARN("[tracker_patterns] So, the node will be stopped gently :)");

    exit(0);

}


bool TrackerPatterns::check_odom_file( const char *odometryPathFile,
                                       const double currentTimeStamp) 
{
    // Opening odometry file
    std::ifstream odom_file;
    odom_file.open( odometryPathFile ); 
    if ( odom_file.is_open() ) {
        ROS_INFO("[tracker_patterns] Opening: %s",odometryPathFile);
    }
    else {
        ROS_ERROR("[tracker_patterns] NOT OPEN: %s",odometryPathFile);
        ROS_ERROR("[tracker_patterns] ERROR, Cannot open odometry file. ");
        ROS_ERROR("[tracker_patterns] Heading_adjst and Distance_adjst without checking odom. ");
        return true;
    }

    // Time Stamp separating integer part and decimal part 
    int currentTimeStampTrunc = currentTimeStamp;
    double currentTimeStampDecim = currentTimeStamp-currentTimeStampTrunc;

    // Time Stamp integer part to string 
    std::stringstream buffer;
    buffer << currentTimeStampTrunc;
    std::string currentTimeStampTruncStr = buffer.str();

    // Reading all the lines looking for the TimeStamp
    std::string line;
    for(unsigned int curLine = 0; getline(odom_file, line); curLine++) {

        if (line.find( currentTimeStampTruncStr ) != std::string::npos) {

            int dummy, tSs, x, y, theta;
            std::string tSm;
            // Reading columns
            std::istringstream ss(line);
            ss >> dummy >> dummy >> dummy >> tSs >> tSm >> dummy >> dummy >> dummy >> x 
               >> y >> dummy >> theta >> dummy >> dummy;

            // Processing Time Stamp milliseconds from string to double
            tSm="0."+tSm;
            double tSmDouble = std::atof(tSm.c_str());

            // Checking period. If there is no robot movement in an interval of 0.5s
            // then we have heading or distance adjustment 
            if (tSmDouble >= currentTimeStampDecim and tSmDouble < currentTimeStampDecim +0.5) {
                //TODO: Check odom values of all interval
                //TODO: return true or false
            }

std::cout<< currentTimeStampTrunc<<":"<<currentTimeStampDecim << "  " << tSs <<":"<<tSm<< std::endl;

        }
    }

    odom_file.close();
    
    
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



