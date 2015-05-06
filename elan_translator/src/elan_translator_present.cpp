#include "ros/ros.h"
#include "data_parser/Parse.h"
#include "data_parser/DataParsed.h"


/**
 *  This node calls the parser rosservice, once for each input file 
 *  and publishes all the Annotations that appears just before an 
 *  object/region/ws user presentation with no more of 10 seconds of difference
 */

int main(int argc, char **argv)
{

    /// Checking entrance
    ros::init(argc, argv, "elan_translator");
    if (argc < 3) {
        ROS_ERROR("[elan_translator] usage: elan_translator PATH_FILE(s) FILE_1 FILE_2 .. FILE_N");
        return 1;
    }

    /// Init basic stuff
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    ros::ServiceClient client = n.serviceClient<data_parser::Parse>("parse_data");
    data_parser::Parse srv;

    /// Calling ROS service to parse files
    data_parser::DataParsed dataXmlFile[argc-2];
    for (int i=0;i<argc-2;++i) {
        srv.request.path = (std::string)argv[1] + (std::string)argv[i+2];
        if (client.call(srv)) {
            ROS_INFO("[elan_translator] Calling parse_data_srv");
        }
        else {
            ROS_ERROR("[elan_translator] Failed to call service parse_data_srv");
            return 1;
        }
        dataXmlFile[i]=srv.response.data;
    }

    /// Find "usr_present" annotations 
    data_parser::AnnotationList usrPresentAnnotations[argc-2];
    for (int i=0;i<argc-2;++i) {
        int iterator_i=0;
        bool trobat=false;
        while (iterator_i<dataXmlFile[i].data.size() and !trobat) {
            if (dataXmlFile[i].data[iterator_i].id=="usr_present") {
                usrPresentAnnotations[i] = dataXmlFile[i].data[iterator_i];
                trobat=true;
            }
            iterator_i++;
        }
    }

    /// Creating publisher
    ros::Publisher annotation_pub = n.advertise<data_parser::DataParsed>("annotations",10);
    // Waiting one second to get ready the publisher 
    ROS_INFO("[elan_translator] Waiting 0.5s to get ready the publisher");
    ros::Duration(0.5).sleep();
    ROS_INFO("[elan_translator] Start Publishing!"); 

    /// Fill msg taking into account usr_present annotations in all the files
    // For all the files parsed
    for (int i=0; i<argc-2; i++) {
        // While ros OK and still are present annotations in [i] file parsed
        while (ros::ok() and usrPresentAnnotations[i].list.size()>0) {
    
            // Topic message
            data_parser::DataParsed pub_msg;
            pub_msg.data.resize(dataXmlFile[i].data.size());
    
            /// Filling msg, with all different kind of annotations
            // For all annotations in [i] file parsed
            for (int j=0;j<dataXmlFile[i].data.size();j++) {
                  
                const int tenSeconds=10000;
                bool noMoar=false;
                /// Obtaining the last annotation in 10s before presentation
                while (dataXmlFile[i].data[j].list.size() > 0 and !noMoar) {
                    if (usrPresentAnnotations[i].list[0].tini-tenSeconds <= 
                      dataXmlFile[i].data[j].list[0].tend and 
                      usrPresentAnnotations[i].list[0].tend >=
                      dataXmlFile[i].data[j].list[0].tini ) {

                        // INSERT!!!
                        // Temp vars to Fill pub_msg
                        data_parser::Annotation tmpAnnotation;
                        tmpAnnotation.text=dataXmlFile[i].data[j].list[0].text;
                        tmpAnnotation.tini=dataXmlFile[i].data[j].list[0].tini;
                        tmpAnnotation.tend=dataXmlFile[i].data[j].list[0].tend;
    
                        data_parser::AnnotationList tmpAnnotationList;
                        tmpAnnotationList.list.resize(1);
                        tmpAnnotationList.list[0]=tmpAnnotation;
                        tmpAnnotationList.id=dataXmlFile[i].data[j].id;
    
                        // Fill pub_msg
                        pub_msg.data[j]=tmpAnnotationList;
    
                        // Delete 1st element
                        dataXmlFile[i].data[j].list.erase(dataXmlFile[i].data[j].list.begin());
    
                    }
                    // if the element is farther than 10s of the presentation
                    // just delete it and continue searching
                    else if (usrPresentAnnotations[i].list[0].tini-tenSeconds >
                        dataXmlFile[i].data[j].list[0].tend) {
                        // Delete 1st element
                        dataXmlFile[i].data[j].list.erase(dataXmlFile[i].data[j].list.begin());
                    } 
                    // No more elements in the list, so skip the bucle 
                    else if (usrPresentAnnotations[i].list[0].tend <
                        dataXmlFile[i].data[j].list[0].tini) {
                        noMoar=true;
                    }
    
                }
    
            }
    
            /// Publishing message
            annotation_pub.publish(pub_msg);
           
            usrPresentAnnotations[i].list.erase(usrPresentAnnotations[i].list.begin());
    
            ros::spinOnce();
            loop_rate.sleep();
    
        }

    }


    /// Publishing last message and closing the node
    data_parser::DataParsed last_pub_msg;
    last_pub_msg.data.resize(1);
    last_pub_msg.data[0].id="NoMOAR!";
    annotation_pub.publish(last_pub_msg);
    
    ROS_WARN("[elan_translator] Nothing else to publish!!!");
    ROS_WARN("[elan_translator] So, the node is stopped gently :)");
    return 0;

}

