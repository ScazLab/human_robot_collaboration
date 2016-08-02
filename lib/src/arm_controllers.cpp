#include "robot_interface/arm_controllers.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;

/*
    BoardStateSensing error at start
    Drop token faster
    Flag option for turning off display windows when run as part of baxterTictactoe
    Get rid of imageScreen node error and show something else via boardScheme
    Error-checking when cellsDefinitionAuto does not see board
*/

/**************************************************************************/
/*                              MoveToRest                                */
/**************************************************************************/

// Public
MoveToRest::MoveToRest(string limb): ROSThreadImage(limb) {}
MoveToRest::~MoveToRest(){}

// Protected
void MoveToRest::InternalThreadEntry()
{
    // wait for endpoint callback
    while(ros::ok())
    {
        if(!(_curr_position.x == 0 && _curr_position.y == 0 && _curr_position.z == 0))
        {       
            break;
        }
    }

    PoseStamped req_pose_stamped;
    
    req_pose_stamped.header.frame_id = "base";
    setPosition(   req_pose_stamped.pose, 0.292391, getLimb() == "left" ? 0.611039 : -0.611039, 0.181133);
    setOrientation(req_pose_stamped.pose, 0.028927, 0.686745, 0.00352694, 0.726314);

    while(ros::ok())
    {
        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        // joint_cmd.names
        setJointNames(joint_cmd);
        joint_cmd.command.resize(7);
        // joint_cmd.angles
        joint_cmd.command[0] = getLimb() == "left" ? 1.1508690861110316   : -1.3322623142784817;
        joint_cmd.command[1] = getLimb() == "left" ? -0.6001699832601681  : -0.5786942522297723;
        joint_cmd.command[2] = getLimb() == "left" ? -0.17449031462196582 : 0.14266021327334347;
        joint_cmd.command[3] = getLimb() == "left" ? 2.2856313739492666   : 2.2695245756764697 ;
        joint_cmd.command[4] = getLimb() == "left" ? 1.8680051044474626   : -1.9945585194480093;
        joint_cmd.command[5] = getLimb() == "left" ? -1.4684031092033123  : -1.469170099597255 ;
        joint_cmd.command[6] = getLimb() == "left" ? 0.1257864246066039   : -0.011504855909140603;

        publish(joint_cmd);

        ros::spinOnce();
        ros::Rate(100).sleep();
 
        if(hasPoseCompleted(_curr_pose, req_pose_stamped.pose, "loose")) 
        {
            break;
        }
    }

    setState(REST);
    pthread_exit(NULL);  
}  

/**************************************************************************/
/*                         PickUpToken                               */
/**************************************************************************/

// Public
PickUpToken::PickUpToken(string limb): ROSThreadImage(limb), Gripper(limb)
{
    namedWindow("[PickUpToken] Raw", WINDOW_NORMAL);    
    namedWindow("[PickUpToken] Processed", WINDOW_NORMAL);
    namedWindow("[PickUpToken] Rough", WINDOW_NORMAL);
    resizeWindow("[PickUpToken] Raw",       700, 500);
    resizeWindow("[PickUpToken] Processed", 700, 500);
    resizeWindow("[PickUpToken] Rough",     700, 500);
}
PickUpToken::~PickUpToken()
{
    destroyWindow("[PickUpToken] Raw");
    destroyWindow("[PickUpToken] Processed"); 
    destroyWindow("[PickUpToken] Rough");
}

// Protected
void PickUpToken::InternalThreadEntry()
{
    // wait for IR sensor callback
    while(ros::ok())
    {
        if(!(get_curr_range() == 0 && get_curr_min_range() == 0 && get_curr_max_range() == 0))
        {
            break; 
        }

        ros::spinOnce();
        ros::Rate(100).sleep();
    }

    // wait for image callback
    while(ros::ok())
    {
        if(!_curr_img_empty) break;
    }

    hoverAboveTokens(POS_HIGH);
    gripToken();
    hoverAboveTokens(POS_LOW);

    setState(PICK_UP);
    pthread_exit(NULL);  
}

// Private
typedef vector<vector<cv::Point> > Contours;

void PickUpToken::gripToken()
{
    cv::Point2d offset;
    // check if token is present before starting movement loop 
    // (prevent gripper from colliding with play surface)
    checkForToken(offset);

    PoseStamped req_pose_stamped;
    ros::Time start_time = ros::Time::now();                
    cv::Point2d prev_offset(0.540, 0.540);

    while(ros::ok())
    {
        processImage(offset);
        ros::Time now_time = ros::Time::now();

        req_pose_stamped.header.frame_id = "base";

        // move incrementally towards token
        setPosition(req_pose_stamped.pose, 
                    prev_offset.x + 0.07 * offset.x,
                    prev_offset.y + 0.07 * offset.y,
                    0.375 + /*(-0.05)*/ -0.08 * (now_time - start_time).toSec());

        prev_offset.x = prev_offset.x + 0.07 * offset.x; 
        prev_offset.y = prev_offset.y + 0.07 * offset.y;

        setOrientation(req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);

        vector<double> joint_angles;
        getJointAngles(req_pose_stamped,joint_angles);

        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        setJointNames(joint_cmd);
        joint_cmd.command.resize(7);

        for(int i = 0; i < 7; i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        publish(joint_cmd);

        ros::spinOnce();
        ros::Rate(100).sleep();
        
        // if(_curr_position.z < -0.05) break;

        if(hasCollided("strict")) 
        {
            break;
        }
    }
    gripObject();
}   

void PickUpToken::checkForToken(cv::Point2d &offset)
{
    ros::Time start_time = ros::Time::now();

    while(ros::ok())
    {
        processImage(offset);

        if(!(offset.x == 0 && offset.y == 0) || (ros::Time::now() - start_time).toSec() > 1)
        {
            break;
        }
    }

    while(ros::ok())
    {
        if(!(offset.x == 0 && offset.y == 0)) {break;}

        // loop halts until a key is pressed
        ROS_WARN("No token detected by hand camera. Place token and press ENTER");
        char c = cin.get();
        processImage(offset);
    }
}

void PickUpToken::processImage(cv::Point2d &offset)
{
    Mat black, blue, token_rough, token, board; 
    Contours contours;
    int board_y;

    isolateBlack(black);
    isolateBoard(black.clone(), board, board_y);

    isolateBlue(blue);
    isolateToken(blue.clone(), board_y, token_rough, contours);
    setOffset(contours, offset, token);

    imshow("[PickUpToken] Raw", _curr_img.clone());
    imshow("[PickUpToken] Processed", token_rough);
    imshow("[PickUpToken] Rough", blue);
    // imshow("[PickUpToken] Final", token);

    waitKey(30);
}

void PickUpToken::isolateBlue(Mat &output)
{
    Mat hsv;

    pause();  // don't remove these prints or it will crash
    pthread_mutex_lock(&_mutex_img); 
    // convert image color format from BGR to HSV  
    cvtColor(_curr_img, hsv, CV_BGR2HSV);
    pthread_mutex_unlock(&_mutex_img);   

    inRange(hsv, Scalar(60,90,10), Scalar(130,256,256), output);
}

void PickUpToken::isolateBlack(Mat &output)
{
    Mat gray;

    pause();  // don't remove these prints or it will crash
    pthread_mutex_lock(&_mutex_img);   
    cvtColor(_curr_img, gray, CV_BGR2GRAY);
    pthread_mutex_unlock(&_mutex_img);  

    threshold(gray, output, 55, 255, cv::THRESH_BINARY_INV);
}

void PickUpToken::isolateBoard(Mat input, Mat &output, int &board_y)
{
    output = Mat::zeros(_curr_img_size, CV_8UC1);

    vector<cv::Vec4i> hierarchy; // captures contours within contours 
    Contours contours;

    // find outer board contours
    findContours(input, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double largest = 0, next_largest = 0;
    int largest_index = 0, next_largest_index = 0;

    // iterate through contours and keeps track of contour w/ 2nd-largest area
    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i], false) > largest)
        {
            next_largest = largest;
            next_largest_index = largest_index;
            largest = contourArea(contours[i], false);
            largest_index = i;
        }
        else if(next_largest < contourArea(contours[i], false) && contourArea(contours[i], false) < largest)
        {
            next_largest = contourArea(contours[i], false);
            next_largest_index = i;
        }
    }

    output = Mat::zeros(_curr_img_size, CV_8UC1);

    // contour w/ 2nd largest area is most likely the inner board
    vector<cv::Point> contour = contours[next_largest_index];

    drawContours(output, contours, next_largest_index, Scalar(255,255,255), CV_FILLED);

    // find the lowest y-coordinate of the board; to be used as a cutoff point above which
    // all contours are ignored (e.g token contours that are above low_y are already placed
    // on the board and should not be picked up)
    int low_y = _curr_img_size.height;
    int x_min = (contours[0])[0].x;
    int x_max = 0;    

    for(int i = 0; i < contour.size(); i++)
    {
        if(contour[i].y < low_y) low_y = contour[i].y;
        if(contour[i].x < x_min) x_min = contour[i].x;
        if(contour[i].x > x_max) x_max = contour[i].x;
    }

    // if width of the contour is narrower than 275, 2nd largest contour
    // is NOT the board (and board is out of the image's view). Hence,
    // no cutoff point needs to be specified
    if(x_max - x_min > 275) {
        board_y = low_y;
    }
    else 
    {
        board_y = _curr_img_size.height;
    }

    line(output, cv::Point(0, board_y), cv::Point(_curr_img_size.width, board_y), cv::Scalar(130,256,256), 5);
}

void PickUpToken::isolateToken(Mat input, int board_y, Mat &output, Contours &contours)
{
    Contours raw_contours, clean_contours, apx_contours, gripper_contours;
    findContours(input, raw_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int gripper_area = -1;
    int gripper_index = 0;

    // find gripper contours. gripper contours are always attached to bottom part of image
    // (Note that imshow will show the bottom (x=0) part inverted and 
    // on top of the display window). if there are multiple contours that contain points w/ x=0
    // the gripper contour is the contour with the largest area as a combination of the contours
    // of the gripper AND  a token fragment will always be larger than just a token fragment
    for(int i = 0; i < raw_contours.size(); i++)
    {
        vector<cv::Point> contour = raw_contours[i];
        for(int j = 0; j < contour.size(); j++)
        {
            if(contour[j].y == 1)
            {
                if(gripper_area == -1) 
                {
                    gripper_area = contourArea(contour, false);
                    gripper_index = i;
                }
                else if(contourArea(contour, false) > gripper_area)
                {
                    gripper_area = contourArea(contour, false);
                    gripper_index = i;
                }
                break;
            }
        }
    }

    // remove gripper contour
    raw_contours.erase(raw_contours.begin() + gripper_index);

    // remove contours that have areas that are too small (noise) and
    // contours that do not have an approx. triangular shape (not token fragment)
    int largest_index = 0, largest_area = 0;
    for(int i = 0; i < raw_contours.size(); i++)
    {
        bool is_triangle = true;
        vector<cv::Point> contour;
        approxPolyDP(raw_contours[i], contour, 0.11 * arcLength(raw_contours[i], true), true);

        if(contour.size() != 3) is_triangle = false;

        if(contourArea(raw_contours[i]) > 200 && is_triangle == true)
        {
            apx_contours.push_back(contour);
            clean_contours.push_back(raw_contours[i]);
        }
    }

    // remove contours that are inside the board (e.g token placed on a cell)
    for(int i = 0; i < clean_contours.size(); i++) 
    {
        bool within_board = false;
        vector<cv::Point> contour = clean_contours[i];
        for(int j = 0; j < contour.size(); j++)
        {
            cv::Point pt = contour[j];
            if(pt.y > board_y)
            {
                within_board = true;
                break;
            }
        }

        if(within_board == false) 
        {
            (contours).push_back(contour);
        }
    }

    output = Mat::zeros(_curr_img_size, CV_8UC1);
    for(int i = 0; i < (contours).size(); i++)
    {
        drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
    }

    line(output, cv::Point(0, board_y), cv::Point(_curr_img_size.width, board_y), cv::Scalar(130,256,256));
}              

void PickUpToken::setOffset(Contours contours, cv::Point2d &offset, Mat &output)
{
    output = Mat::zeros(_curr_img_size, CV_8UC1);

    // when hand camera is blind due to being too close to token, go straight down;
    if(contours.size() < 2)
    {
        offset = cv::Point2d(0,0);
    }
    else if(contours.size() <= 4)
    {
        // find highest and lowest x and y values from token triangles contours
        // to find x-y coordinate of top left token edge and token side length
        double y_min = (contours[0])[0].y;
        double x_min = (contours[0])[0].x;
        double y_max = 0;
        double x_max = 0;

        for(int i = 0; i < contours.size(); i++)
        {
            vector<cv::Point> contour = contours[i];
            for(int j = 0; j < contour.size(); j++)
            {
                if(y_min > contour[j].y) y_min = contour[j].y;
                if(x_min > contour[j].x) x_min = contour[j].x;
                if(y_max < contour[j].y) y_max = contour[j].y;
                if(x_max < contour[j].x) x_max = contour[j].x;
            }
        }

        // reconstruct token's square shape
        Rect token(x_min, y_min, y_max - y_min, y_max - y_min);
        rectangle(output, token, Scalar(255,255,255), CV_FILLED);

        // find and draw the center of the token and the image
        double x_mid = x_min + ((x_max - x_min) / 2);
        double y_mid = y_min + ((y_max - y_min) / 2);
        circle(output, cv::Point(x_mid, y_mid), 3, Scalar(0, 0, 0), CV_FILLED);

        circle(output, cv::Point(_curr_img_size.width / 2, _curr_img_size.height / 2), 3, Scalar(180, 40, 40), CV_FILLED);

        double token_area = (x_max - x_min) * (y_max - y_min);

        (offset).x = (/*4.7807*/ 5 / token_area) * (x_mid - (_curr_img_size.width / 2)); 
        (offset).y = (/*4.7807*/ 5 / token_area) * ((_curr_img_size.height / 2) - y_mid) - 0.0075; /*distance between gripper center and camera center*/         
    }
}

/**************************************************************************/
/*                          ScanBoard                                */
/**************************************************************************/

// Public
ScanBoard::ScanBoard(string limb): ROSThreadImage(limb)
{
    namedWindow("[ScanBoard] Rough",     WINDOW_NORMAL);
    namedWindow("[ScanBoard] Processed", WINDOW_NORMAL);
}
ScanBoard::~ScanBoard()
{
    destroyWindow("[ScanBoard] Rough");
    destroyWindow("[ScanBoard] Processed");
}

vector<geometry_msgs::Point> ScanBoard::getOffsets() { return _offsets; }

// Protected
void ScanBoard::InternalThreadEntry()
{
    hoverAboveBoard();

    // wait for image callback
    while(ros::ok())
    {
        if(!_curr_img_empty) break;

        ros::spinOnce();
        ros::Rate(100).sleep();
    }

    scan();
    hoverAboveTokens(POS_HIGH);

    setState(SCANNED);
    pthread_exit(NULL);
}

// Private
void ScanBoard::hoverAboveBoard()
{
    goToPose(0.575, 0.100, 0.445, 0.99962, -0.02741, 0, 0);
}

void ScanBoard::scan()
{
    float dist;
    setDepth(&dist);
    hoverAboveBoard();
    processImage("run", dist);   
}

void ScanBoard::setDepth(float *dist)
{
    geometry_msgs::Point init_pos = _curr_position;

    ros::Time start_time = ros::Time::now();                

    // move downwards until collision with surface
    while(ros::ok())
    {
        PoseStamped req_pose_stamped;
        req_pose_stamped.header.frame_id = "base";

        setPosition(req_pose_stamped.pose, 
                    init_pos.x,
                    init_pos.y,
                    init_pos.z + (-0.07) * (ros::Time::now() - start_time).toSec());

        setOrientation(req_pose_stamped.pose, 0.99962, -0.02741, 0, 0);

        vector<double> joint_angles;
        getJointAngles(req_pose_stamped,joint_angles);

        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        setJointNames(joint_cmd);
        joint_cmd.command.resize(7);

        for(int i = 0; i < 7; i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        publish(joint_cmd);
        ros::spinOnce();
        ros::Rate(100).sleep();
     
        if(hasCollided("loose")) 
        {
            break;
        }
    }

    // offset to account for height difference between IR camera and tip of vacuum gripper
    *dist = init_pos.z - _curr_position.z + 0.04;
}

void ScanBoard::processImage(string mode, float dist)
{
    ros::Time start_time = ros::Time::now();

    while(ros::ok())
    {
        Contours contours;
        vector<cv::Point> centroids, board_corners, cell_to_corner;
        
        Mat binary, board;

        int board_area;

        isolateBlack(&binary);
        isolateBoard(&contours, &board_area, &board_corners, binary.clone(), &board);

        waitKey(3);

        if(contours.size() == 9)
        {
            setOffsets(board_area, contours, dist, &board, &centroids);
            // imshow("[ScanBoard] Processed", board);
        
            if(offsetsReachable() && mode == "run"){
                cout << "[Scan Board] Board is positioned correctly! Proceed with game" << endl;
                break;
            }
            else if(!offsetsReachable()) {
                cout << "[Scan Board] Please move board within reachable zone" << endl;
                setZone(contours, dist, board_corners, &centroids, &cell_to_corner);

                // calls to IK solver in setZone takes too long; makes the image update
                // very slow and hard for users to calibrate board position, which is why
                // and inner loop is needed
                ros::Time start = ros::Time::now();
                int interval = 10;
                while(ros::ok())
                {
                    Mat zone = _curr_img.clone();

                    line(zone, centroids[0] + cell_to_corner[0], centroids[2] + cell_to_corner[1], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[2] + cell_to_corner[1], centroids[8] + cell_to_corner[3], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[0] + cell_to_corner[0], centroids[6] + cell_to_corner[2], cv::Scalar(0,0,255), 1);
                    line(zone, centroids[6] + cell_to_corner[2], centroids[8] + cell_to_corner[3], cv::Scalar(0,0,255), 1);

                    if((ros::Time::now() - start).toSec() > interval)
                    {
                        vector<cv::Point> temp_centroids, temp_board_corners;
                        isolateBlack(&binary);
                        isolateBoard(&contours, &board_area, &temp_board_corners, binary.clone(), &board);
                        if(contours.size() == 9)
                        {
                            setOffsets(board_area, contours, dist, &board, &temp_centroids);
                        }
                        if(offsetsReachable()) 
                        {
                            break;
                        }
                        interval += 5;
                    }

                    imshow("[ScanBoard] Rough", zone);
                    
                    waitKey(3);
                }
            }
        }

        imshow("[ScanBoard] Processed", board);
    }
}

void ScanBoard::isolateBlack(Mat * output)
{
    Mat gray;
    pause();
    pthread_mutex_lock(&_mutex_img);   
    cvtColor(_curr_img, gray, CV_BGR2GRAY);
    pthread_mutex_unlock(&_mutex_img);   
    threshold(gray, *output, 55, 255, cv::THRESH_BINARY);
}

void ScanBoard::isolateBoard(Contours * contours, int * board_area, vector<cv::Point> * board_corners, Mat input, Mat * output)
{
    *output = Mat::zeros(_curr_img_size, CV_8UC1);

    vector<cv::Vec4i> hierarchy; // captures contours within contours 

    findContours(input, *contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double largest = 0, next_largest = 0;
    int largest_index = 0, next_largest_index = 0;

    // iterate through contours and keeps track of contour w/ 2nd-largest area
    for(int i = 0; i < (*contours).size(); i++)
    {
        if(contourArea((*contours)[i], false) > largest)
        {
            next_largest = largest;
            next_largest_index = largest_index;
            largest = contourArea((*contours)[i], false);
            largest_index = i;
        }
        else if(next_largest < contourArea((*contours)[i], false) && contourArea((*contours)[i], false) < largest)
        {
            next_largest = contourArea((*contours)[i], false);
            next_largest_index = i;
        }
    }

    *board_area = contourArea((*contours)[next_largest_index], false);

    drawContours(*output, *contours, next_largest_index, Scalar(255,255,255), CV_FILLED, 8, hierarchy);

    findContours(*output, *contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    largest = 0;
    largest_index = 0;

    // iterate through contours and keeps track of contour w/ largest area
    for(int i = 0; i < (*contours).size(); i++)
    {
        if(contourArea((*contours)[i], false) > largest)
        {
            largest = contourArea((*contours)[i], false);
            largest_index = i;
        }
    } 

    vector<cv::Point> board_outline = (*contours)[largest_index];

    /* Set board corners and board area*/
    double y_min = board_outline[0].y;
    double x_min = board_outline[0].x;
    double y_max = 0;
    double x_max = 0;

    for(int i = 0; i < board_outline.size(); i++)
    {
        if(y_min > board_outline[i].y) y_min = board_outline[i].y;
        if(x_min > board_outline[i].x) x_min = board_outline[i].x;
        if(y_max < board_outline[i].y) y_max = board_outline[i].y;
        if(x_max < board_outline[i].x) x_max = board_outline[i].x;
    }
    
    (*board_corners).push_back(cv::Point(x_max, y_max));
    (*board_corners).push_back(cv::Point(x_min, y_max));
    (*board_corners).push_back(cv::Point(x_max, y_min));
    (*board_corners).push_back(cv::Point(x_min, y_min));

    // remove outer board contours
    (*contours).erase((*contours).begin() + largest_index);

    for(int i = 0; i < (*contours).size(); i++)
    {
        if(contourArea((*contours)[i], false) < 200)
        {
            (*contours).erase((*contours).begin() + i);
        } 
    }

    for(int i = 0; i < (*contours).size(); i++)
    {
        drawContours(*output, *contours, i, Scalar(255,255,255), CV_FILLED);
    }
}

bool ScanBoard::descendingX(vector<cv::Point> i, vector<cv::Point> j) 
{
    double x_i = moments(i, false).m10 / moments(i, false).m00;
    double x_j = moments(j, false).m10 / moments(j, false).m00;

    return x_i > x_j;
}

void ScanBoard::setOffsets(int board_area, Contours contours, float dist, Mat *output, vector<cv::Point> *centroids)
{
    cv::Point center(_curr_img_size.width / 2, _curr_img_size.height / 2);

    circle(*output, center, 3, Scalar(180,40,40), CV_FILLED);
    cv::putText(*output, "Center", center, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));

    for(int i = contours.size(); i >= 3; i -= 3)
    {
        std::sort(contours.begin() + (i - 3), contours.begin() + i, descendingX);        
    }

    _offsets.resize(9);
    (*centroids).resize(9);
    for(int i = contours.size() - 1; i >= 0; i--)
    {
        double x = moments(contours[i], false).m10 / moments(contours[i], false).m00;
        double y = moments(contours[i], false).m01 / moments(contours[i], false).m00;
        cv::Point centroid(x,y);  

        (*centroids)[i] = centroid;

        // cv::putText(*output, intToString(i), centroid, cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(180,40,40));
        // circle(*output, centroid, 2, Scalar(180,40,40), CV_FILLED);
        line(*output, centroid, center, cv::Scalar(180,40,40), 1);

        _offsets[i].x = (centroid.y - center.y) * 0.0025 * dist + 0.04;  
        _offsets[i].y = (centroid.x - center.x) * 0.0025 * dist;
        _offsets[i].z = dist - 0.065;
    }
}

void ScanBoard::setZone(Contours contours, float dist, vector<cv::Point> board_corners, vector<cv::Point> * centroids, vector<cv::Point> * cell_to_corner)
{
    (*cell_to_corner).resize(4);

    // calculate offset between the center of corner cells and the corners of the board
    (*cell_to_corner)[0] = cv::Point(board_corners[0].x - (*centroids)[0].x, board_corners[0].y - (*centroids)[0].y);
    (*cell_to_corner)[1] = cv::Point(board_corners[1].x - (*centroids)[2].x, board_corners[1].y - (*centroids)[2].y);
    (*cell_to_corner)[2] = cv::Point(board_corners[2].x - (*centroids)[6].x, board_corners[2].y - (*centroids)[6].y);
    (*cell_to_corner)[3] = cv::Point(board_corners[3].x - (*centroids)[8].x, board_corners[3].y - (*centroids)[8].y);

    // if the centroid of a corner cell is reachable, 
    // iterate and check if a location 10 pixels further from arm is still reachable
    // to establish a boundary of how far Baxter's arm can reach
    while(pointReachable((*centroids)[0], dist)) {(*centroids)[0].x += 10.0;}
    while(pointReachable((*centroids)[2], dist)) {(*centroids)[2].x -= 10.0;}
    while(pointReachable((*centroids)[6], dist)) {(*centroids)[6].x += 10.0;}
    while(pointReachable((*centroids)[8], dist)) {(*centroids)[8].x -= 10.0;}

    // if the centroid of a corner cell is unreachable, 
    // iterate and check if a location 10 pixels closer is reachable
    while(!pointReachable((*centroids)[0], dist)) {(*centroids)[0].x -= 5.0;}
    while(!pointReachable((*centroids)[2], dist)) {(*centroids)[2].x += 5.0;}
    while(!pointReachable((*centroids)[6], dist)) {(*centroids)[6].x -= 5.0;}
    while(!pointReachable((*centroids)[8], dist)) {(*centroids)[8].x += 5.0;}
}

bool ScanBoard::offsetsReachable()
{
    for(int i = 0; i < 9; i++)
    {
        PoseStamped req_pose_stamped;
        req_pose_stamped.header.frame_id = "base";
        setPosition(req_pose_stamped.pose, 
                    _curr_position.x + _offsets[i].x, 
                    _curr_position.y + _offsets[i].y, 
                    _curr_position.z - _offsets[i].z);
        setOrientation(req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);

        vector<double> joint_angles;
        getJointAngles(req_pose_stamped,joint_angles);
        
        // if IK solver returns a joint angles solution with all zeros, 
        // then no solution was found
        bool all_zeros = true;
        for(int j = 0; j < joint_angles.size(); j++)
        {
            if(joint_angles[j] != 0) 
            {
                all_zeros = false;
                break;
            }
        }    
        if(all_zeros) return false;    
    }
    return true;
}

bool ScanBoard::pointReachable(cv::Point centroid, float dist)
{
    // convert image location into real world pose coordinates
    cv::Point center(_curr_img_size.width / 2, _curr_img_size.height / 2);

    geometry_msgs::Point offset;

    offset.x = (centroid.y - center.y) * 0.0025 * dist + 0.04;  
    offset.y = (centroid.x - center.x) * 0.0025 * dist;
    offset.z = dist - 0.085;

    PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "base";
    setPosition(pose_stamped.pose, 
                0.575 + offset.x,
                0.100 + offset.y,
                0.445 - offset.z);
    setOrientation(pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);

    vector<double> joint_angles;
    return getJointAngles(pose_stamped,joint_angles) ? false : true;
}

/**************************************************************************/
/*                         PutDownToken                              */
/**************************************************************************/

// Public
PutDownToken::PutDownToken(string limb): ROSThreadImage(limb), Gripper(limb) {}        
PutDownToken::~PutDownToken() {}

void PutDownToken::setCell(int cell) {_cell = cell;}
void PutDownToken::setOffsets(vector<geometry_msgs::Point> offsets) {_offsets = offsets;}

// Protected
void PutDownToken::InternalThreadEntry()
{
    hoverAboveBoard();
    hoverAboveCell();
    ros::Duration(0.8).sleep();
    releaseObject();
    hoverAboveBoard();
    hoverAboveTokens(POS_HIGH);

    setState(PUT_DOWN);
    pthread_exit(NULL);  
}  

// Private
void PutDownToken::hoverAboveCell()
{
    goToPose(0.575 + _offsets[_cell - 1].x,
             0.100 + _offsets[_cell - 1].y, 
             0.445 - _offsets[_cell - 1].z,
             VERTICAL_ORIENTATION_LEFT_ARM);
}

void PutDownToken::hoverAboveBoard()
{
    goToPose(0.575 + _offsets[4].x,
             0.100 + _offsets[4].y, 
             0.445 - _offsets[4].z,
             VERTICAL_ORIENTATION_LEFT_ARM);
}

/**************************************************************************/
/*                            ArmController                               */
/**************************************************************************/

ArmController::ArmController(string limb): _limb(limb)
{
    _rest_class = new MoveToRest(_limb);
    _pick_class = new PickUpToken(_limb);
    _scan_class = new ScanBoard(_limb);
    _put_class  = new PutDownToken(_limb);
}

ArmController::~ArmController()
{
    delete _rest_class;
    delete _pick_class;
    delete _scan_class;
    delete _put_class;
}

int ArmController::getState()
{
    float len_time = 0;
    int state = 0;

    // find class with the most recent state change
    // and set ArmController's new state to that
    // class' state
    if(_rest_class->getState().time > len_time) 
    {
        len_time = _rest_class->getState().time; 
        state = _rest_class->getState().state;
    }

    if(_pick_class->getState().time > len_time) 
    {
        len_time = _pick_class->getState().time; 
        state = _pick_class->getState().state;
    }

    if(_scan_class->getState().time > len_time) 
    {
        len_time = _scan_class->getState().time; 
        state = _scan_class->getState().state;
    }

    if(_put_class->getState().time > len_time) 
    {
        len_time = _put_class->getState().time; 
        state = _put_class->getState().state;            
    }

    return state;
}

void ArmController::moveToRest() {_rest_class->startInternalThread();}

void ArmController::pickUpToken() {_pick_class->startInternalThread();}

void ArmController::scanBoard() {_scan_class->startInternalThread();}

void ArmController::putDownToken(int cell) 
{
    _put_class->setOffsets(_scan_class->getOffsets());
    _put_class->setCell(cell);
    _put_class->startInternalThread();
}    

