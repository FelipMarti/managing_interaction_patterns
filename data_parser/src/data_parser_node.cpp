/**
 *      ROS-service to parse an ELAN XML file given a path. This ELAN file contains all 
 *      the annotations made by the user in a HRI show episode.   
 *      This service returns all the data parsed in a ROS msg
 *
 *      Lunds tekniska högskola | LTH 2015
 *      Felip Marti Carrillo
 *
 *      MIT License (MIT) 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */


#include "ros/ros.h"
#include "data_parser/DataParsed.h"
#include "data_parser/Parse.h"
#include <tinyxml.h>


/**
 *  Function to parse the action time, init time and ending time. 
 */
void parse_time(TiXmlElement* root, std::string timeIni, std::string timeEnd, 
                unsigned int &tIni, unsigned int &tEnd) 
{
    //ROOT
    for(TiXmlElement* elem = root->FirstChildElement();
        elem !=0; elem = elem->NextSiblingElement()) {
    
        //TIME_ORDER
        std::string elemName = elem->Value();
        if (elemName == "TIME_ORDER") {
            //Searching TIME_SLOT_ID
            bool exit=false;
            TiXmlElement* child = elem->FirstChildElement();
            while (child!=0 or !exit) {
    
                //Comparing TIME_VALUE attribute to obtain time in ms
                std::string elemName = child->Attribute("TIME_SLOT_ID"); 
                if (timeIni==elemName) {
                    tIni=atof(child->Attribute("TIME_VALUE"));
                }
                if (timeEnd==elemName) {
                    tEnd=atof(child->Attribute("TIME_VALUE"));
                    exit=true;
                }

                child = child->NextSiblingElement();

            }
        }

   }   
 
}


/**
 *  Function to parse all the data info. 
 */
bool parse_data(std::string path, data_parser::DataParsed &dataParsed )
{

    /// Reading XML file to obtain data
    TiXmlDocument xml_doc(path);
    if (!xml_doc.LoadFile()) {
        ROS_ERROR("[data_parser_service]: ERROR when loading XML file");
        ROS_ERROR("[data_parser_service]: path=\"%s\"",path.c_str());
        return false;
    }
    TiXmlElement* root = xml_doc.FirstChildElement();
    if (root == NULL) {
        ROS_ERROR("[data_parser_service]: Failed to load file: No root element");
        return false;
    }

    ROS_INFO("[data_parser_service]: Parsing XML file");
    ROS_INFO("[data_parser_service]: path=\"%s\"",path.c_str());

    /// Extracting data Annotations from XML file
    dataParsed.data.resize(0);
    int numListAnnotation=0;
    //ROOT
    for(TiXmlElement* elem = root->FirstChildElement();
        elem !=0; elem = elem->NextSiblingElement()) {

        //TIERS
        std::string elemName = elem->Value();
        if (elemName == "TIER") {
            data_parser::AnnotationList tmpAnnList;
            tmpAnnList.id=elem->Attribute("TIER_ID");
            tmpAnnList.list.resize(0);
            dataParsed.data.push_back(tmpAnnList);

            //ANNOTATION
            for(TiXmlElement* child = elem->FirstChildElement();
                child !=0; child = child->NextSiblingElement()) {

                //ALIGNABLE_ANNOTATION
                TiXmlElement* subchild = child->FirstChildElement();
                if (subchild) {

                    //ANNOTATION_VALUE, here we have the useful data 
                    TiXmlElement* subsubchild = subchild->FirstChildElement();
                    if (subsubchild) {
                        data_parser::Annotation tmpAnn;
                        // To aviod core dump problems :s
                        // when the string is empty!
                        if (subsubchild->GetText()>0) {
                            tmpAnn.text=subsubchild->GetText();
                        }
                        else {
                            tmpAnn.text="";
                        }
                        //Time slot extraction
                        const char* timeIni = subchild->Attribute("TIME_SLOT_REF1");
                        const char* timeEnd = subchild->Attribute("TIME_SLOT_REF2");
                        //Time value extraction
                        parse_time(root, timeIni, timeEnd, tmpAnn.tini, tmpAnn.tend);

                        //Filling data
                        dataParsed.data[numListAnnotation].list.push_back(tmpAnn);

                    }

                }

            }
            //Change of kind of annotation
            numListAnnotation++;
        }
    }

    ROS_INFO("[data_parser_service]: XML file Parsed!");
    return true;

}


bool parse_srv(data_parser::Parse::Request  &req, data_parser::Parse::Response &res) {
    
    return parse_data(req.path, res.data);
    
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_parser_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("parse_data", parse_srv);
    ROS_INFO("[data_parser_service] Ready to parse your data!");

    ros::spin();

    return 0;
}

