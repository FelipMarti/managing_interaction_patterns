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


#include "interaction_learner_node.h"


InteractionLearner::InteractionLearner (void)
{

    // Init Subscriber
    this->data_sub = this->n.subscribe("BN_vars", 1,
                            &InteractionLearner::read_data_callback, this);

    // Init Bayesian Network learning variables
    lastCommandData.resize(0);    
    usrAnnounceData.resize(0);
    usrGestureData.resize(0);
    headingAdjData.resize(0);
    distanceAdjData.resize(0);
    category.resize(0);

    subCounter = 0;
    
}


InteractionLearner::~InteractionLearner (void)
{
}


void InteractionLearner::read_data_callback
                            (const interaction_monitor::BayesianNetworkVariable& msg) {

    /// Generating Bayesian Network
    if (msg.category==-1) {

        subCounter++;
        ROS_INFO("[Interaction_Learner] I've heard something, #%d",subCounter);

        // Fill vars with data
        dataBayesianNetwork.SetNumberOfRecords(category.size());
        for (int i=0; i<category.size(); i++) {
            dataBayesianNetwork.SetInt(0,i,lastCommandData[i]);
            dataBayesianNetwork.SetInt(1,i,usrAnnounceData[i]);
            dataBayesianNetwork.SetInt(2,i,usrGestureData[i]);
            dataBayesianNetwork.SetInt(3,i,headingAdjData[i]);
            dataBayesianNetwork.SetInt(4,i,distanceAdjData[i]);
            dataBayesianNetwork.SetInt(5,i,category[i]);
        }

        ROS_INFO("[Interaction_Learner] with %d Vars and %d Annotations",
                 dataBayesianNetwork.GetNumberOfVariables(), 
                 dataBayesianNetwork.GetNumberOfRecords());


        // Match the data set and the network:
        std::vector<DSL_datasetMatch> matches;
        std::string err;
        if (dataBayesianNetwork.MatchNetwork(net, matches, err) != DSL_OKAY) {
            ROS_ERROR("[Interaction_Learner] Cannot match network... exiting.");
            ROS_ERROR("[Interaction_Learner] %s",err.c_str());
            exit(1);
        }
        else {
            ROS_INFO("[Interaction_Learner] BN matched successfully :)");
        }

        // Train BN
        ROS_INFO("[Interaction_Learner] Training BN");
        DSL_network result;
        DSL_nb naive;
        naive.classVariableId = "category";
        if (naive.Learn(dataBayesianNetwork,result)!=DSL_OKAY) {
            ROS_ERROR("[Interaction_Learner] Learning failed");
            exit(1);
        }
        else {
            pathBN=pathBN+"naivebayes.xdsl";
            result.WriteFile(pathBN.c_str());      
            ROS_INFO("[Interaction_Learner] BN generated in:");
            ROS_INFO("[Interaction_Learner] %s",pathBN.c_str());
            ROS_WARN("[Interaction_Learner] So, the node is stopped gently :)");
            exit(0);
        }

    }
    // Filling Bayesian Network variables
    else { 
        lastCommandData.push_back(msg.last_cmd);  
        usrAnnounceData.push_back(msg.announce);  
        usrGestureData.push_back(msg.gesture);    
        headingAdjData.push_back(msg.head_adj);   
        distanceAdjData.push_back(msg.dist_adj);  
        category.push_back(msg.category);          
    }

}


int InteractionLearner::Main (const char* path, const char* file)
{

    // Path to write the new BN
    pathBN=path;
    
    // Open the BN network
    std::string full_path = (std::string) path + (std::string) file;
    if (net.ReadFile(full_path.c_str(), DSL_XDSL_FORMAT) != DSL_OKAY) {
        ROS_ERROR("[Interaction_Learner] Cannot read network... Exiting!");
        exit(1);
    }
    else {
        ROS_INFO("[Interaction_Learner] BN opened successfully:");
        ROS_INFO("[Interaction_Learner] %s",full_path.c_str());
    }

    // Preparing dataset for the BN
    set_nomenclature_to_dataset(); 

    // Wait for callbacks
    ros::spin();

}


/**
 *  Setting the correct nomenclature to match the dataset with the 
 *  Bayesian Network 
 */
void InteractionLearner::set_nomenclature_to_dataset()
{

    std::vector<std::string> vecNames;

    /// Node (var) lastCommand
    dataBayesianNetwork.AddIntVar("lastCommand");
    // states names
    vecNames.resize(0);
    vecNames.push_back("back");     // back     => 0
    vecNames.push_back("follow");   // follow   => 1
    vecNames.push_back("forward");  // forward  => 2
    vecNames.push_back("stop");     // stop     => 3
    vecNames.push_back("turn");     // turn     => 4
    vecNames.push_back("none");     // none     => 5
    dataBayesianNetwork.SetStateNames(0, vecNames);

    /// Node (var) usrAnnounceData 
    dataBayesianNetwork.AddIntVar("usrAnnounce");
    // states names
    vecNames.resize(0);
    vecNames.push_back("no");       // no  => 0
    vecNames.push_back("yes");      // yes => 1
    dataBayesianNetwork.SetStateNames(1, vecNames);

    /// Node (var) usrGesture 
    dataBayesianNetwork.AddIntVar("usrGesture");
    // states names
    vecNames.resize(0);
    vecNames.push_back("fingertip_point");  // fingertip_point  => 0
    vecNames.push_back("hand_point");       // hand_point       => 1
    vecNames.push_back("hold_item");        // hold_item        => 2
    vecNames.push_back("sweep_wave");       // sweep_wave       => 3
    vecNames.push_back("touch_full_hand");  // touch_full_hand  => 4
    vecNames.push_back("none");             // none             => 5
    dataBayesianNetwork.SetStateNames(2, vecNames);

    /// Node (var) headingAdjData
    dataBayesianNetwork.AddIntVar("headingAdj");
    // states names
    vecNames.resize(0);
    vecNames.push_back("no");       // no  => 0
    vecNames.push_back("yes");      // yes => 1
    dataBayesianNetwork.SetStateNames(3, vecNames);

    /// Node (var) distanceAdjData 
    dataBayesianNetwork.AddIntVar("distanceAdj");
    // states names
    vecNames.resize(0);
    vecNames.push_back("no");       // no  => 0
    vecNames.push_back("yes");      // yes => 1
    dataBayesianNetwork.SetStateNames(4, vecNames);

    /// Node (var) Category
    dataBayesianNetwork.AddIntVar("category");
    // states names
    vecNames.resize(0);
    vecNames.push_back("object");       // object    => 0 
    vecNames.push_back("region");       // region    => 1
    vecNames.push_back("workspace");    // workspace => 2
    vecNames.push_back("unknown");      // unknown   => 3
    dataBayesianNetwork.SetStateNames(5, vecNames);

}


int main(int argc, char** argv) 
{
    
    ros::init(argc, argv, "interaction_learner");
    if (argc != 3) {
        ROS_ERROR("[interaction_learner] usage: interaction_learner PATH Bayesian_Network");
        exit(1);
    }


    InteractionLearner foo;
    return foo.Main(argv[1], argv[2]);

}



/**
 *  Dictionary to define commands because some annotations have typographic errors.
 *
 *  back    => 0
 *  follow  => 1
 *  forward => 2
 *  stop    => 3
 *  turn    => 4
 *  none    => 5
 *
 */


/**
 *  Dictionary to define gestures because some annotations have typographic errors.
 *
 *  fingertip_point => 0
 *  hand_point      => 1
 *  hold_item       => 2
 *  sweep_wave      => 3
 *  touch_full_hand => 4
 *  none            => 5
 *
 */


/**
 *  Dictionary to define the presentation category in object, region or workspace.
 *  Besides, unknown category is defined when it was not confirmed by the robot, or
 *  could cause ambiguity
 *
 *  object      => 0
 *  region      => 1
 *  workspace   => 2
 *  unknown     => 3
 *
 */
