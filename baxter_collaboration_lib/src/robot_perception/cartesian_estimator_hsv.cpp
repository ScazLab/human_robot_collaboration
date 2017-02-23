#include "robot_perception/cartesian_estimator_hsv.h"

using namespace std;

/************************************************************************************/
/*                               SEGMENTED OBJECT HSV                               */
/************************************************************************************/
SegmentedObjHSV::SegmentedObjHSV(vector<double> _size, hsvColorRange _col) :
                                 SegmentedObj(_size), col(_col)
{

}

SegmentedObjHSV::SegmentedObjHSV(string _name, vector<double> _size, int _area_thres,
                                 hsvColorRange _col) :
                                 SegmentedObj(_name, _size, _area_thres), col(_col)
{

}

bool SegmentedObjHSV::detectObject(const cv::Mat& _in, cv::Mat& _out)
{
    cv::Mat out_thres(_in.rows, _in.cols, CV_32FC1, 0.0);
    return detectObject(_in, _out, out_thres);
}

bool SegmentedObjHSV::detectObject(const cv::Mat& _in, cv::Mat& _out, cv::Mat& _out_thres)
{
    // ROS_INFO("Detecting object: %s", toString().c_str());

    cv::Mat img_hsv;
    cv::cvtColor(_in, img_hsv, CV_BGR2HSV); //Convert the captured frame from BGR to HSV

    Contours           contours;
    Contours      filt_contours;
    vector<cv::Vec4i> hierarchy;

    cv::Mat img_thres = hsvThreshold(img_hsv, col);

    // Some morphological operations to remove noise and clean up the image
    for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());
    for (int i = 0; i < 4; ++i) dilate(img_thres, img_thres, cv::Mat());
    for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());

    // Find contours
    cv::findContours(img_thres, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Let's filter out contours that are too small to be an object
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // ROS_INFO("IDX %lu Col %s Contour area %g", i, col.toString().c_str(),
        //                                            contourArea(contours[i], false));
        if( int(contourArea(contours[i], false)) > area_threshold )
        {
            filt_contours.push_back(contours[i]);
        }
    }

    img_thres = cv::Mat::zeros(_in.size(), CV_8UC1);
    cv::drawContours(img_thres, filt_contours, -1, cv::Scalar::all(255), CV_FILLED, 8);

    cv::bitwise_or(_out_thres, img_thres.clone(), _out_thres);

    // If there no contours any more, the object is not there
    if (filt_contours.size() == 0)
    {
        setIsThere(false);
        return false;
    }

    Contour detected_obj;

    // At this stage, there should be only a contour in the vector of contours.
    // Strategy 1: If this is not the case, let's merge all the available contours
        for (size_t i = 0; i < filt_contours.size(); ++i)
        {
            detected_obj.insert(detected_obj.end(), filt_contours[i].begin(), filt_contours[i].end());
        }

    // Strategy 2: If this is not the case, let's pick the contour with the biggest area
        // int largest_area=0;
        // int largest_contour_idx=-1;

        // for( size_t i = 0; i< filt_contours.size(); i++ )
        // {
        //     double a = contourArea(filt_contours[i], false);  //  Find the area of contour

        //     if( a > largest_area )
        //     {
        //         largest_area = a;
        //         largest_contour_idx = i;                // Store the index of largest contour
        //     }
        // }

        // detected_obj = filt_contours[largest_contour_idx];

    rect = minAreaRect(cv::Mat(detected_obj));
    setIsThere(true);

    return true;
}

SegmentedObjHSV::operator std::string()
{
    return std::string(getName() + " [" + toString(  size[0]) + " "
                                        + toString(  size[1]) + "],"
                                   " [" + toString(col.H.min) + " "
                                        + toString(col.H.max) + "],"
                                   " [" + toString(col.S.min) + " "
                                        + toString(col.S.max) + "],"
                                   " [" + toString(col.V.min) + " "
                                        + toString(col.V.max) + "]");
}

SegmentedObjHSV::~SegmentedObjHSV()
{

}

/************************************************************************************/
/*                             CARTESIAN ESTIMATOR HSV                              */
/************************************************************************************/
CartesianEstimatorHSV::CartesianEstimatorHSV(string  _name) : CartesianEstimator(_name)
{
    XmlRpc::XmlRpcValue objects_db;
    if(!_n.getParam("/"+getName()+"/objects_db", objects_db))
    {
        ROS_INFO("No objects' database found in the parameter server. "
                 "Looked up param is %s", ("/"+getName()+"/objects_db").c_str());
    }
    else
    {
        addObjects(objects_db);
        printObjectDB();
    }
}

bool CartesianEstimatorHSV::addObject(string _name, double _h, double _w, hsvColorRange _hsv)
{
    vector<double> size;

    // Let's put the longer size first
    if (_h > _w)
    {
        size.push_back(_h);
        size.push_back(_w);
    }
    else
    {
        size.push_back(_w);
        size.push_back(_h);
    }

    objs.push_back(new SegmentedObjHSV(_name, size, getAreaThreshold(), _hsv));

    return true;
}

bool CartesianEstimatorHSV::addObjects(vector<string> _names, cv::Mat _o,
                                       vector<hsvColorRange> _hsvs)
{
    if (_o.rows != int(_hsvs.size()))
    {
        ROS_ERROR("Rows of the matrix should be equal to the size of the hsv color vector!");
        return false;
    }

    clearObjs();

    bool res = true;

    for (int i = 0; i < _o.rows; ++i)
    {
        res = res & addObject(_names[i], _o.at<float>(i, 0), _o.at<float>(i, 1), _hsvs[i]);
    }

    return res;
}

bool CartesianEstimatorHSV::addObjects(XmlRpc::XmlRpcValue _params)
{
    ROS_ASSERT(_params.getType()==XmlRpc::XmlRpcValue::TypeStruct);
    ROS_WARN_COND(_params.size() == 0, "No objects available in the parameter server!");

    bool res = true;

    for (XmlRpc::XmlRpcValue::iterator i=_params.begin(); i!=_params.end(); ++i)
    {
        ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeStruct);

        ROS_ASSERT(i->second.size()>=2);

        for (XmlRpc::XmlRpcValue::iterator j=i->second.begin(); j!=i->second.end(); ++j)
        {
            // ROS_ASSERT(j->first.getType()==XmlRpc::XmlRpcValue::TypeString);
            if (j->first=="size")
            {
                ROS_ASSERT(j->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(j->second[0].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(j->second[1].getType()==XmlRpc::XmlRpcValue::TypeDouble);
            }
            if (j->first=="HSV")
            {
                ROS_ASSERT(j->second.getType()==XmlRpc::XmlRpcValue::TypeStruct);
                ROS_ASSERT(j->second["H"].getType()==XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(j->second["S"].getType()==XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(j->second["V"].getType()==XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(j->second["H"][0].getType()==XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(j->second["H"][1].getType()==XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(j->second["S"][0].getType()==XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(j->second["S"][1].getType()==XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(j->second["V"][0].getType()==XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(j->second["V"][1].getType()==XmlRpc::XmlRpcValue::TypeInt);

            }
        }

        res = res & addObject(static_cast<string>(i->first.c_str()),
                              static_cast<double>(i->second["size"][0]),
                              static_cast<double>(i->second["size"][1]),
                              hsvColorRange(i->second["HSV"]["H"], i->second["HSV"]["S"], i->second["HSV"]["V"]));
    }

    return res;
}
