#include "node.h"

bool finish_flag = false;

PARSE::PARSE(ros::NodeHandle nh,ros::NodeHandle np)
:nh_(nh),
 np_(np)
{
    //订阅odom和激光的话题
    sub_ar_track = nh_.subscribe("tag_detections", 10, &PARSE::tag_detections_mark,this);
    sub_orb_pose =nh_.subscribe("orb_slam2_rgbd/pose", 30, &PARSE::orb_pose,this);
    // sub_darknet =nh_.subscribe("/darknet_ros_msgs/bounding_boxes", 30, &PARSE::darknet_Bbox,this);

    // ros::Publisher pub_ar_pose = nh_.advertise<geometry_msgs::PoseStamped>("ar_pose", 10);
    // ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    std::cout << "init finish" << std::endl;
   
}

PARSE::~PARSE(){}

bool PARSE::get_ar_trans(Eigen::Vector3d& trans_ar,Eigen::Quaterniond quat_object,string ar_name)
{
    
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/map", ar_name,ros::Time(0), transform);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute ar pose, skipping scan (%s)", e.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    trans_ar << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();
    // trans_ar << ar_pose.pose.position.x,ar_pose.pose.position.y,ar_pose.pose.position.z;
    quat_object.w() = transform.getRotation().getW();
    quat_object.x() = transform.getRotation().getX();
    quat_object.y() = transform.getRotation().getY();
    quat_object.z() = transform.getRotation().getZ();
   
    cout << "quat_ar " << quat_object.toRotationMatrix() << endl;

    return true;
}



void PARSE::tag_detections_mark(const apriltag_ros::AprilTagDetectionArray& msg)
{
    int t=0;
    int sum=0;
    // tf::TransformListener listener;

    std::stringstream ss;

    for(const apriltag_ros::AprilTagDetection& ar_marker : msg.detections)
    {
        ss.str("");
        ss << "tag_"<< ar_marker.id[0];

        // cout << ss.str() << endl;


        Eigen::Vector3d trans_object;
        Eigen::Quaterniond quat_object;
        // if(PARSE::get_ar_trans(trans_object,quat_object,ss.str()) == false)
        // {
        //     std::cout <<"Failed to get ar Pose"<<std::endl;
        //     return ;
        // }

        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/map", ss.str(),ros::Time(0), transform);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute ar pose, skipping scan (%s)", e.what());
            ros::Duration(1.0).sleep();
            return ;
        }
        trans_object << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();

        quat_object.w() = transform.getRotation().getW();
        quat_object.x() = transform.getRotation().getX();
        quat_object.y() = transform.getRotation().getY();
        quat_object.z() = transform.getRotation().getZ();

        string name;
        Vector9d S_object;

        // ROS_INFO("%d",  ar_marker.id[0]);

        // std::cout << "trans_object:" << ar_marker.id[0] << std::endl << trans_object << std::endl;
        // cout << "quat_object : " << quat_object.toRotationMatrix() << endl;

        T_base_to_apri = Eigen::Isometry3d::Identity();
        T_base_to_apri.rotate(quat_object.toRotationMatrix());
        T_base_to_apri.pretranslate(trans_object);
        // cout << "quat_object : " << T_base_to_apri.matrix() << endl;

        // TV
        if(ar_marker.id[0] == 0 && TV == false)
        {
            TV = true;
            name = "TV";

            Eigen::AngleAxisd rotation_vector(M_PI / 2,Eigen::Vector3d(0,0,0));
            Eigen::Vector3d t2(0.3,0,0);
            Eigen::Vector3d t3(0,0,0.1);
            Eigen::Vector3d t4(0.3,0,0.1);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;


            S_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            
        }
        // desk
        else if(ar_marker.id[0] == 1 && desk == false)
        {
            desk = true;
            name = "desk";

            Eigen::AngleAxisd rotation_vector(M_PI / 2,Eigen::Vector3d(0,0,0));
            Eigen::Vector3d t2(1,0,0);
            Eigen::Vector3d t3(0,-2,0);
            Eigen::Vector3d t4(1,-2,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            cout << "T_base_to_apri :" << T_base_to_apri.matrix() << endl;
            cout << "desk : " << S_object << endl;
        }
        // computer
        else if(ar_marker.id[0] == 2 && computer == false)
        {
            computer = true;
            name = "computer";

            Eigen::AngleAxisd rotation_vector(M_PI / 2,Eigen::Vector3d(0,0,0));
            Eigen::Vector3d t2(0.1,0,0);
            Eigen::Vector3d t3(0,-0.3,0);
            Eigen::Vector3d t4(0.1,-0.3,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];
        }
        // chair
        else if(ar_marker.id[0] == 3 && chair == false)
        {
            chair = true;
            name = "chair";

            Eigen::AngleAxisd rotation_vector(M_PI / 2,Eigen::Vector3d(0,0,0));
            Eigen::Vector3d t2(0.2,0,0);
            Eigen::Vector3d t3(0,-0.2,0);
            Eigen::Vector3d t4(0.2,-0.2,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            cout << "chair : " << S_object << endl; 
        }
        // air_conditioner
        else if(ar_marker.id[0] == 4 && air_conditioner == false)
        {
            air_conditioner = true;
            name = "air_conditioner";

            Eigen::AngleAxisd rotation_vector(M_PI / 2,Eigen::Vector3d(0,0,0));
            Eigen::Vector3d t2(0.1,0,0);
            Eigen::Vector3d t3(0,0,0.1);
            Eigen::Vector3d t4(0.1,0,0.1);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];
        }
        
        else 
        {
            if(TV==true && desk==true && computer==true && chair==true && air_conditioner==true && finish_flag != true)
            {
                Json::Value root;
                root["name"] = "meeting_room";

                boost::property_tree::ptree knowledgegraph = PARSE::loadPoseFile("/home/ybg/knowledgegraph.json");
                boost::property_tree::ptree knowledgegraph_object = knowledgegraph.get_child("object");

                Json::Value attribute;


                for(auto iter = Sbox.begin();iter != Sbox.end(); iter++)
                {
                    string name_object = iter->first;
                    Vector9d Sbox_object = iter->second;

                    Json::Value object;
                    object["name"] = Json::Value(name_object);
                    object.removeMember("XYZ");
                    object["XYZ"].append(Sbox_object[0]);
                    object["XYZ"].append(Sbox_object[1]);
                    object["XYZ"].append(Sbox_object[8]);

                    map<string,float> intersection_S;
                    float ares = (Sbox_object(6) - Sbox_object(0)) * (Sbox_object(7) - Sbox_object(1));
                    for(auto iter_ = Sbox.begin();iter_ != Sbox.end(); iter_++)
                    {
                        string name_object_ = iter_->first;
                        if(name_object_.compare(name_object))
                        {
                            Vector9d Sbox_object_ = iter_->second;
                            if(Sbox_object[8] > Sbox_object_[8]     // Z > Z_
                            && Sbox_object[0] > min(Sbox_object_[0],Sbox_object_[6])     //Xmax > Xmin
                            && Sbox_object[0] < max(Sbox_object_[0],Sbox_object_[6])     //Xmax > Xmin
                            && Sbox_object[1] > min(Sbox_object_[1],Sbox_object_[7])     //Ymax > Ymin
                            && Sbox_object[1] < max(Sbox_object_[1],Sbox_object_[7]))    //Ymax > Ymin
                            {
                                object["on"] = Json::Value(name_object_);
                            }
                        }
                        BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, knowledgegraph_object)
                        {
                            boost::property_tree::ptree vt = vtt.second;
                            string name_kg = vt.get<string>("name");
                            if(name_object == name_kg )
                            {
                                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, vt)
                                {
                                    if(v.second.get_value<std::string>() != name_kg)
                                    {
                                        attribute[v.first]=Json::Value(v.second.get_value<std::string>());
                                    }
                                }
                                object["attribute"] = attribute;
                            }
                        }
                    }

                    root["object"].append(object);
                }

                finish_flag = true;

                cout << "StyledWriter:" << endl;
                Json::StyledWriter sw;
                cout << sw.write(root) << endl << endl;

                // 输出到文件
                ofstream dataFile;//记录数据
                dataFile.open("/home/ybg/kg.json",ios::out);
                dataFile << sw.write(root);
                dataFile.close();

            }

            break;
        }

        Sbox.insert(std::map<string,Vector9d>::value_type(name,S_object));

        
        
    }
   
    //ROS_INFO("%d", t);
    
}

void PARSE::orb_pose(const geometry_msgs::PoseStamped& pose_msg)
{
    // T_base_to_camera = Eigen::Isometry3d::Identity();

    Eigen::Vector3d trans;
    trans[0] = pose_msg.pose.position.x;
    trans[1] = pose_msg.pose.position.y;
    trans[2] = pose_msg.pose.position.z;

    Eigen::Quaterniond quat;
    quat.x() = pose_msg.pose.orientation.x; 
    quat.y() = pose_msg.pose.orientation.y; 
    quat.z() = pose_msg.pose.orientation.z; 
    quat.w() = pose_msg.pose.orientation.w; 

    // T_base_to_camera.rotate ( quat );
    // T_base_to_camera.pretranslate ( trans );
    // std::cout << "T_base_to_camera:" << std::endl << T_base_to_camera.matrix() << std::endl;
}


// void PARSE::darknet_Bbox(const geometry_msgs::PoseStamped& pose_msg)
// {

// }
