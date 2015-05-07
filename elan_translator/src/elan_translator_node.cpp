/**
 *      This node calls the ROS-service parser for each input file 
 *      and publishes all the Annotations that appear in time intervals of 500ms
 *      
 *      Before publishing those annotations, they are translated from strings to integers
 *      in order to transform from ELAN variables to Bayesian network variables.
 *
 *      The trigger topic will be published with a "true" when the end time of the usr_present 
 *      annotation is finished. This trigger will be used to train or perform inference 
 *      with all the immediate annotations in the subscriber nodes.
 *
 *      When there is no more data and before closing the node the trigger topic 
 *      will publish "false"
 *      
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT) 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */

#include "elan_translator_node.h"


ElanTranslator::ElanTranslator (void)
{

    /// Init publishers
    last_command_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_last_cmd",10);
    announce_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_announce",10);
    gesture_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_gesture",10);
    heading_adj_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_heading_adj",10);
    dist_adj_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_dist_adj",10);
    category_pub = n.advertise<interaction_monitor::AnnotationVariable>("ELAN_category",10);
    trigger_pub = n.advertise<std_msgs::Bool>("ELAN_trigger",1);

    /// Filling dictionaries
    write_category_dictionary();
    write_gesture_dictionary();
    write_command_dictionary();

}


ElanTranslator::~ElanTranslator (void)
{
}

int ElanTranslator::Main (int argc, char **argv) 
{

    ros::Rate loop_rate(2000);

    /// Init service
    ros::ServiceClient client = n.serviceClient<data_parser::Parse>("parse_data");
    data_parser::Parse srv;

    /// Calling ROS service to parse files
    dataXmlFile.resize(argc-2);
    for (int i=0;i<argc-2;++i) {
        srv.request.path = (std::string)argv[1] + (std::string)argv[i+2];
        if (client.call(srv)) {
            ROS_INFO("[ELAN_translator] Calling parse_data_srv");
        }
        else {
            ROS_ERROR("[ELAN_translator] Failed to call service parse_data_srv");
            return 1;
        }
        dataXmlFile[i]=srv.response.data;
    }

    ROS_INFO("[ELAN_translator] Waiting 0.5s to get ready the publisher");
    ros::Duration(0.5).sleep();
    ROS_INFO("[ELAN_translator] Start Publishing!");

    /// Main Loop
    // For all files entered
    for (int j=0; j<argc-2; j++) {
        
        int currentTime = 0;
        // Var to change file in case there are no more things to publish
        int checkIfNothingElseToPublish=0;

        while (ros::ok() and checkIfNothingElseToPublish != dataXmlFile[j].data.size()) {
            
            checkIfNothingElseToPublish=0;

            // For all different tiers in a file
            for (int i=0;i<dataXmlFile[j].data.size();i++) {

                // Check no empty tier (vector) to avoid segmentation fault
                if (dataXmlFile[j].data[i].list.size() > 0) {                

                    // Check init time to publish data or not
                    if (dataXmlFile[j].data[i].list[0].tini <= currentTime) {

                        //Temp vars to Fill pub_msg
                        interaction_monitor::AnnotationVariable tmpAnnotation;
                        tmpAnnotation.tini=dataXmlFile[j].data[i].list[0].tini;
                        tmpAnnotation.tend=dataXmlFile[j].data[i].list[0].tend;
                
                        /// Publishing
                        // Checking only 5 tiers and publish 6 annotations
                        if (dataXmlFile[j].data[i].id == "usr_cmd") {
                            tmpAnnotation.value = 
                                        CommandCategory[dataXmlFile[j].data[i].list[0].text];
                            last_command_pub.publish(tmpAnnotation);
                        }
                        else if (dataXmlFile[j].data[i].id == "usr_announce") {
                            tmpAnnotation.value = 1;    //yes 
                            announce_pub.publish(tmpAnnotation);
                        }
                        else if (dataXmlFile[j].data[i].id == "usr_gesture") {
                            tmpAnnotation.value = 
                                        GestureCategory[dataXmlFile[j].data[i].list[0].text];
                            gesture_pub.publish(tmpAnnotation);
                        }
                        else if (dataXmlFile[j].data[i].id == "usr_mov") {

                            std::size_t found1=dataXmlFile[j].data[i].list[0].text.find("adj");
                            std::size_t found2=dataXmlFile[j].data[i].list[0].text.find("adjustment");
                            if ( found1!=std::string::npos or found2!=std::string::npos ) {
                                tmpAnnotation.value = 1;    //yes
                                heading_adj_pub.publish(tmpAnnotation);
                            }
                            
                            found1=dataXmlFile[j].data[i].list[0].text.find("closer");
                            found2=dataXmlFile[j].data[i].list[0].text.find("further");
                            if ( found1!=std::string::npos or found2!=std::string::npos ) {
                                tmpAnnotation.value = 1;   //yes
                                dist_adj_pub.publish(tmpAnnotation);
                            }

                        } 
                        else if (dataXmlFile[j].data[i].id == "usr_present") {
                            tmpAnnotation.value = 
                                        ObjCategory[dataXmlFile[j].data[i].list[0].text]; 
                            category_pub.publish(tmpAnnotation);
                        }

                    }
                    // Check end time to erase element
                    if (dataXmlFile[j].data[i].list[0].tend <= currentTime) {
                        if (dataXmlFile[j].data[i].id == "usr_present") {
                            std_msgs::Bool hola;
                            hola.data = true;
                            trigger_pub.publish(hola);
                        }
                        dataXmlFile[j].data[i].list.erase(dataXmlFile[j].data[i].list.begin());
    
                    }


                }
                else {
                    checkIfNothingElseToPublish++;
                }

            }

            /// Publishing data parsed in intervals of 500ms
            currentTime+=500;
    
            ros::spinOnce();
            loop_rate.sleep();

        }

        

    }

    // Publishing false in trigger to indicate that there is no more data
    std_msgs::Bool hola;
    hola.data = false;
    trigger_pub.publish(hola);

    ROS_WARN("[ELAN_translator] No more data!");
    ROS_WARN("[ELAN_translator] So, the node will be stopped gently :)");
    exit(0);

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "elan_translator_node");
    if (argc < 3) {
        ROS_ERROR(
        "[ELAN_translator] usage: elan_translator_node PATH_FILE(s) FILE_1 FILE_2 .. FILE_N");
        exit(1);
    }

    ElanTranslator foo;
    return foo.Main(argc, argv);

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
void ElanTranslator::write_command_dictionary()
{
    CommandCategory["back"]=0;          //back
    CommandCategory["follow"]=1;        //follow
    CommandCategory["forward"]=2;       //forward
    CommandCategory["stop"]=3;          //stop
    CommandCategory["stopp"]=3;         //stop
    CommandCategory["turn_around"]=4;   //turn
    CommandCategory["turn_left"]=4;     //turn
    CommandCategory["turn_right"]=4;    //turn
    CommandCategory["none"]=5;          //none
}


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
void ElanTranslator::write_gesture_dictionary()
{
    GestureCategory["fignertip_point"]=0;       //fingertip_point
    GestureCategory["fingertip_point"]=0;       //fingertip_point
    GestureCategory["hand_point"]=1;            //hand_point
    GestureCategory["hand_point?"]=1;           //hand_point
    GestureCategory["hold item"]=2;             //hold_item
    GestureCategory["hold_item"]=2;             //hold_item
    GestureCategory["sweep_wave"]=3;            //sweep_wave
    GestureCategory["touch_full_hand"]=4;       //touch_full_hand
    GestureCategory["touch_item_full_hand"]=4;  //touch_full_hand
    GestureCategory["touch_item_full_hand / hold_item"]=4;  //touch_full_hand
    GestureCategory["touch_item_full_hand?"]=4; //touch_full_hand
    GestureCategory["none"]=5;                  //none
}


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
void ElanTranslator::write_category_dictionary()
{
    ObjCategory["LUCAS_entrance"]=2;      //workspace
    ObjCategory["LUCAS_room"]=1;          //region
    ObjCategory["Xerox_machine"]=2;       //workspace
    ObjCategory["armchair"]=0;            //object
    ObjCategory["backboard"]=2;           //workspace
    ObjCategory["basin"]=2;               //workspace
    ObjCategory["basket"]=0;              //object
    ObjCategory["beer"]=0;                //object
    ObjCategory["big_table"]=2;           //workspace
    ObjCategory["bin"]=0;                 //object
    ObjCategory["blue_chair"]=0;          //object
    ObjCategory["board"]=3;               //unknown
    ObjCategory["book"]=0;                //object
    ObjCategory["books"]=0;               //object
    ObjCategory["bookshelf"]=2;           //workspace
    ObjCategory["bookshelves"]=2;         //workspace
    ObjCategory["bottle"]=0;              //object
    ObjCategory["bottle_of_water"]=0;     //object
    ObjCategory["box"]=0;                 //object
    ObjCategory["buttons"]=3;             //unknown
    ObjCategory["cable"]=0;               //object
    ObjCategory["chair"]=0;               //object
    ObjCategory["chairs"]=0;              //object
    ObjCategory["chart"]=2;               //workspace
    ObjCategory["coffe_machine"]=3;       //unknown
    ObjCategory["coffe_maker"]=2;         //workspace
    ObjCategory["coffee"]=0;              //object
    ObjCategory["coffee-machine"]=3;      //unknown
    ObjCategory["coffee_machine"]=3;      //unknown
    ObjCategory["coffee_maker"]=3;        //unknown
    ObjCategory["coffee_mug"]=0;          //object
    ObjCategory["coffee_room"]=1;         //region
    ObjCategory["coffeemachine"]=3;       //unknown
    ObjCategory["coffeemaker"]=2;         //workspace
    ObjCategory["computer"]=2;            //workspace
    ObjCategory["computer_keyboard"]=0;   //object
    ObjCategory["computer_monitor"]=2;    //workspace
    ObjCategory["conference_room"]=1;     //region
    ObjCategory["conference_table"]=2;    //workspace
    ObjCategory["control_remote"]=0;      //object
    ObjCategory["copy_machine"]=2;        //workspace
    ObjCategory["copy_room"]=1;           //region
    ObjCategory["copy_room_copy_machine_paper"]=2;//workspace
    ObjCategory["copying_machine"]=2;     //workspace
    ObjCategory["copying_room"]=2;        //workspace
    ObjCategory["cup"]=0;                 //object
    ObjCategory["cupboard"]=2;            //workspace
    ObjCategory["desk"]=3;                //unknown
    ObjCategory["desk_chair"]=0;          //object
    ObjCategory["door"]=2;                //workspace
    ObjCategory["drink"]=0;               //object
    ObjCategory["dust bin"]=3;            //unknown
    ObjCategory["dustbin"]=0;             //object
    ObjCategory["entrance"]=2;            //workspace
    ObjCategory["entrance_door"]=2;       //workspace
    ObjCategory["entrance_to_LUCAS"]=2;   //workspace
    ObjCategory["entrance_to_the_LUCAS"]=2;       //workspace
    ObjCategory["entrance_to_the_LUCAS_room"]=2;  //workspace
    ObjCategory["eraser"]=0;              //object
    ObjCategory["file"]=0;                //object
    ObjCategory["fridge"]=2;              //workspace
    ObjCategory["garbage_can"]=0;         //object
    ObjCategory["glass"]=0;               //object
    ObjCategory["glass_of_water"]=0;      //object
    ObjCategory["glasses"]=0;             //object
    ObjCategory["hallway"]=1;             //region
    ObjCategory["hanger"]=2;              //workspace
    ObjCategory["helmet"]=0;              //object
    ObjCategory["internet_cable"]=0;      //object
    ObjCategory["kensington_lock"]=0;     //object
    ObjCategory["key"]=0;                 //object
    ObjCategory["key_chain"]=0;           //object
    ObjCategory["keys"]=0;                //object
    ObjCategory["kitchen"]=3;             //unknown
    ObjCategory["labtop"]=0;              //object
    ObjCategory["labtop bag"]=0;          //object
    ObjCategory["lamp"]=0;                //object
    ObjCategory["light"]=0;               //object
    ObjCategory["light_switches"]=3;      //unknown
    ObjCategory["lucas_entrance"]=2;      //workspace
    ObjCategory["lunch_room"]=1;          //region
    ObjCategory["markers"]=0;             //object
    ObjCategory["meeting_room"]=1;        //region
    ObjCategory["menu"]=3;                //unknown
    ObjCategory["microwave"]=2;           //workspace
    ObjCategory["microwave_oven"]=2;      //workspace
    ObjCategory["milk"]=0;                //object
    ObjCategory["monitor"]=2;             //workspace
    ObjCategory["mouse"]=0;               //object
    ObjCategory["no_fan"]=3;              //unknown
    ObjCategory["notebook"]=3;            //unknown
    ObjCategory["object_printer"]=0;      //object
    ObjCategory["office"]=1;              //region
    ObjCategory["office_room"]=1;         //region
    ObjCategory["oven"]=2;                //workspace
    ObjCategory["own_arm"]=3;             //unknown
    ObjCategory["paper"]=0;               //object
    ObjCategory["paper_bin"]=0;           //object
    ObjCategory["paper_clip"]=0;          //object
    ObjCategory["papers"]=0;              //object
    ObjCategory["pen"]=0;                 //object
    ObjCategory["phone"]=0;               //object
    ObjCategory["photocopy_machine"]=2;   //workspace
    ObjCategory["photocopying_machine"]=2;//workspace
    ObjCategory["postit"]=0;              //object
    ObjCategory["printer"]=3;             //unknown
    ObjCategory["printer_machine"]=3;     //unknown
    ObjCategory["printer_room"]=2;        //workspace
    ObjCategory["printing_room"]=2;       //workspace
    ObjCategory["projector"]=2;           //workspace
    ObjCategory["refrigerator"]=2;        //workspace
    ObjCategory["remote"]=0;              //object
    ObjCategory["remote_control"]=0;      //object
    ObjCategory["room"]=1;                //region
    ObjCategory["room_4105"]=1;           //region
    ObjCategory["scanning_machine"]=2;    //workspace
    ObjCategory["scissors"]=0;            //object
    ObjCategory["screen"]=3;              //unknown
    ObjCategory["seminar_room"]=1;        //region
    ObjCategory["shelf"]=2;               //workspace
    ObjCategory["sink"]=2;                //workspace
    ObjCategory["small_pen"]=0;           //object
    ObjCategory["small_table"]=2;         //workspace
    ObjCategory["stapler"]=0;             //object
    ObjCategory["table"]=2;               //workspace
    ObjCategory["table_lamp"]=0;          //object
    ObjCategory["tape"]=0;                //object
    ObjCategory["telephone"]=0;           //object
    ObjCategory["towel"]=0;               //object
    ObjCategory["trash_bin"]=0;           //object
    ObjCategory["trashbin"]=0;            //object
    ObjCategory["waiting_room"]=1;        //region
    ObjCategory["wall"]=3;                //unknown
    ObjCategory["water"]=0;               //object
    ObjCategory["water_bottle"]=0;        //object
    ObjCategory["whiteboard"]=2;          //workspace
    ObjCategory["window"]=2;              //workspace
}

