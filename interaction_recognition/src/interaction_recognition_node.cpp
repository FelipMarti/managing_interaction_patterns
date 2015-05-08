/*
 *      This node is used to perform inference in a Bayesian network.
 *      It's subscribed to interaction_monitor /BN_vars topic 
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


#include "interaction_recognition_node.h"


InteractionRecognition::InteractionRecognition (void)
{

    // Init Subscriber
    this->sub = this->n.subscribe("BN_vars", 1,
                        &InteractionRecognition::perform_inference_callback, this);

    // Init Stats
    for (int i=0; i<9; i++) {
         Stats_Results[i]=0; 
    }

    // Max difference to consider 2 objects different
    MAX_DIFF = 0.1;

    // Vector of strings with fails
    statsFails.resize(0);

}


InteractionRecognition::~InteractionRecognition (void)
{
}


void InteractionRecognition::perform_inference_callback
                        (const interaction_monitor::BayesianNetworkVariable& msg)
{

    if (msg.category==-1) {

        print_statistics();
        exit(0);

    }
    
    /// Performing Inference
    ROS_INFO("[Interaction_Recognition] Performing Inference");
    
    // Vars to get the handle of node
    int category = theNet.FindNode("category"); 
    int lastCommand = theNet.FindNode("lastCommand"); 
    int usrAnnounce = theNet.FindNode("usrAnnounce"); 
    int usrGesture = theNet.FindNode("usrGesture"); 
    int headingAdj = theNet.FindNode("headingAdj"); 
    int distanceAdj = theNet.FindNode("distanceAdj"); 

    // Vars to store data
    int lastCommandData=msg.last_cmd;
    int usrAnnounceData=msg.announce;
    int usrGestureData=msg.gesture;   
    int headingAdjData=msg.head_adj;  
    int distanceAdjData=msg.dist_adj; 
    int categoryData=msg.category;     
    
    // Setting evidence
    theNet.GetNode(lastCommand)->Value()->SetEvidence(lastCommandData);
    theNet.GetNode(usrAnnounce)->Value()->SetEvidence(usrAnnounceData);
    theNet.GetNode(usrGesture)->Value()->SetEvidence(usrGestureData);
    theNet.GetNode(headingAdj)->Value()->SetEvidence(headingAdjData);
    theNet.GetNode(distanceAdj)->Value()->SetEvidence(distanceAdjData);

    // Update the network
    theNet.UpdateBeliefs();

    // Get the result values
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(category)->Value());
    DSL_idArray *theNames;
    theNames = theNet.GetNode(category)->Definition()->GetOutcomesNames();

    int objectIndex = theNames->FindPosition("object"); 
    int regionIndex = theNames->FindPosition("region"); 
    int workspaceIndex = theNames->FindPosition("workspace"); 
    int unknownIndex = theNames->FindPosition("unknown"); 

    // Probability of category 
    double P_CategoryIs[4];
    // 0 P_CategoryIsObject
    // 1 P_CategoryIsRegion
    // 2 P_CategoryIsWorkspace
    // 3 P_CategoryIsUnknown

    theCoordinates[0] = objectIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = object)
    P_CategoryIs[0] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = object) = %f",P_CategoryIs[0]);

    theCoordinates[0] = regionIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = region)
    P_CategoryIs[1] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = region) = %f",P_CategoryIs[1]);

    theCoordinates[0] = workspaceIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = workspace)
    P_CategoryIs[2] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = workspace) = %f",P_CategoryIs[2]);

    theCoordinates[0] = unknownIndex;
    theCoordinates.GoToCurrentPosition();
    // get P("category" = unknown)
    P_CategoryIs[3] = theCoordinates.UncheckedValue();
    ROS_INFO("[Interaction_Recognition] P(\"category\" = unknown) = %f",P_CategoryIs[3]);

    ROS_INFO("[Interaction_Recognition] User was presenting category %d", categoryData);


    /// UPDATING STATISTICS
    int CategoryWin=-1;
    double P_CategoryWin=-1;
    for (int i=0; i<4; i++) {
        if (P_CategoryWin < P_CategoryIs[i]) {
            P_CategoryWin = P_CategoryIs[i];
            CategoryWin=i;
        }
    }

    // Difference
    P_CategoryIs[0] = P_CategoryWin-P_CategoryIs[0];
    P_CategoryIs[1] = P_CategoryWin-P_CategoryIs[1];
    P_CategoryIs[2] = P_CategoryWin-P_CategoryIs[2];
    P_CategoryIs[3] = P_CategoryWin-P_CategoryIs[3];

    // Checking results category
    int equalCategory=0;
    std::vector<int> equalCategories;
    for (int i=0; i<4; i++) {
        if (P_CategoryIs[i]<MAX_DIFF) {
            equalCategory++;
            equalCategories.push_back(i);
        }
    }

    // Updating stats
    if (equalCategory==1 and categoryData == CategoryWin) {
        Stats_Results[0]++;     // Good Job!
    }
    else if (equalCategory==1 and categoryData != CategoryWin) {
        if (categoryData == 3) {
            Stats_Results[5]++;     // Nice, Unknown category classified!
        }
        else {
            Stats_Results[1]++;     // Missmatch!!
            std::ostringstream stringStream;
            stringStream << categoryData <<"->"
                         << CategoryWin;
            statsFails.push_back(stringStream.str());
        }
    }
    else if (equalCategory==2) {    // 2 are similar
        if ( P_CategoryIs[categoryData] < MAX_DIFF or categoryData == 3) {
            Stats_Results[2]++;     // Category is among them, or is Unknown
            std::ostringstream stringStream;
            stringStream << categoryData <<" -> "<< equalCategories[0]<<" or "
                         << equalCategories[1];
            statsBetween2.push_back(stringStream.str());
        }
        else {
            Stats_Results[6]++;     // Category NOT among them, FAIL
        }
    }
    else if (equalCategory==3) {    // 3 are similar
        if ( P_CategoryIs[categoryData] < MAX_DIFF or categoryData == 3) {
            Stats_Results[3]++;     // Category is among them, or is Unknown
            std::ostringstream stringStream;
            stringStream << categoryData <<" "<< equalCategories[0]<<", "
                         << equalCategories[1]<<" or " << equalCategories[2];
            statsAmong3.push_back(stringStream.str());
        }
        else {
            Stats_Results[7]++;     // Category NOT among them, FAIL
        }
    }
    else if (equalCategory==4) {    // all are similar
        Stats_Results[4]++;
    }
    else {
        Stats_Results[8]++;         //WTF??
    }

    // Clear the evidence in nodes
    theNet.GetNode(lastCommand)->Value()->ClearEvidence();
    theNet.GetNode(usrAnnounce)->Value()->ClearEvidence();
    theNet.GetNode(usrGesture)->Value()->ClearEvidence();
    theNet.GetNode(headingAdj)->Value()->ClearEvidence();
    theNet.GetNode(distanceAdj)->Value()->ClearEvidence();


    ROS_INFO("[Interaction_Recognition] * * * * ") ;
    
}


int InteractionRecognition::Main (const char* path) {

    // Open BN GENIE file
    if (theNet.ReadFile(path) != 0) {
        ROS_ERROR("[interaction_recognition] Cannot open the Bayesian Network");
        ROS_ERROR("[interaction_recognition] BN=\"%s\"",path);
        return 1;
    }
    else {
        ROS_INFO("[interaction_recognition] Bayesian Network opened successfully");
        ROS_INFO("[interaction_recognition] BN=\"%s\"",path);
    }

    // Wait for callbacks
    ros::spin();

}




void InteractionRecognition::print_statistics()
{

    ROS_WARN("[Interaction_Learner] No more data!");
    ROS_WARN("[Interaction_Learner] So, the node will be stopped gently :)");
    ROS_INFO("[Interaction_Learner] ********** STATISTICS **********");
    ROS_INFO("[Interaction_Learner] %d are OK!!", Stats_Results[0]);
    ROS_INFO("[Interaction_Learner] %d Mismatches!!", Stats_Results[1]);
    for (int i=0; i<statsFails.size(); i++) {
        std::cout<<statsFails[i]<<"; ";
    }
    std::cout<<std::endl;
    ROS_INFO("[Interaction_Learner] %d are Similar between 2", Stats_Results[2]);
    for (int i=0; i<statsBetween2.size(); i++) {
        std::cout<<statsBetween2[i]<<"; ";
    }
    std::cout<<std::endl;
    ROS_INFO("[Interaction_Learner] %d are Similar among 3", Stats_Results[3]);
    for (int i=0; i<statsAmong3.size(); i++) {
        std::cout<<statsAmong3[i]<<"; ";
    }
    std::cout<<std::endl;
    ROS_INFO("[Interaction_Learner] %d are Similar among 4", Stats_Results[4]);
    ROS_INFO("[Interaction_Learner] %d are Unknown classified", Stats_Results[5]);
    ROS_INFO("[Interaction_Learner] %d are Similar between 2, but FAIL", Stats_Results[6]);
    ROS_INFO("[Interaction_Learner] %d are Similar among 3, but FAIL", Stats_Results[7]);
    ROS_INFO("[Interaction_Learner] %d, if > 0, something weird is going on, CHECK CODE!!!", 
                Stats_Results[8]);
    ROS_INFO("[Interaction_Learner] ********** ********** **********");

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "interaction_recognition");
    if (argc != 2) {
        ROS_ERROR("[interaction_recognition] usage: interaction_recognition BAYESIAN_NET");
        exit(1);
    }

    InteractionRecognition foo;
    return foo.Main(argv[1]);

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
