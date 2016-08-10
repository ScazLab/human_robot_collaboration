#ifndef __TTT_CONTROLLERS_H__
#define __TTT_CONTROLLERS_H__

#include "robot_interface/robot_interface.h"
#include "robot_interface/gripper.h"

class MoveToRest : public ROSThreadImage
{
    public:
        MoveToRest(std::string limb);
        ~MoveToRest();

    protected:

        /*
         * moves arm to rest position
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();
};

class PickUpToken : public ROSThreadImage, public Gripper
{
    public:
        PickUpToken(std::string limb);
        ~PickUpToken();

    protected:

        /*
         * picks up token
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;

        /*
         * move arm downwards and suck token upon collision
         * 
         * @param      N/A
         * return     N/A
         */
        void gripToken();

        /*
         * check if hand camera detects token 
         * 
         * @param      Point representing offset between the arm's x-y coordinates
         *            and the token
         * return     N/A
         */
        void checkForToken(cv::Point2d &offset);

        /*
         * identifies token and calculates offset distance required to move hand camera
         * to token
         * 
         * @param      Point representing offset between the arm's x-y coordinates
         *            and the token
         * return     N/A
         */
        void processImage(cv::Point2d &offset);

        /*
         * isolates blue colored object in raw image
         * 
         * @param      Mat displaying blue colored objects in raw image
         * return     N/A
         */
        void isolateBlue(cv::Mat &output);

        /*
         * isolates black colored object in raw image
         * 
         * @param      Mat displaying black colored objects in raw image
         * return     N/A
         */
        void isolateBlack(cv::Mat &output);

        /*
         * isolates board boundaries from image 
         * 
         * @param      input Mat, output Mat displaying board boundaries,
         *            and integer indicating lowest y coordinate of board boundaries
         * return     N/A
         */
        void isolateBoard(cv::Mat input, cv::Mat &output, int &board_y);

        /*
         * isolates token from image
         * 
         * @param      input Mat, output Mat displaying token,
         *            and integer indicating lowest y coordinate of board boundaries,
         *            and contours of blue-colored objects in image
         * return     N/A
         */
        void isolateToken(cv::Mat input, int board_y, cv::Mat &output, Contours &contours);

        /*
         * calculates offset distance from arm to token
         * 
         * @param      token contours, an integer indicating lowest y coordinate of board boundaries,
         *            and contours of blue-colored objects in image, and an output Mat displaying token
         * return     N/A
         */
        void setOffset(Contours contours, cv::Point2d &offset, cv::Mat &output);
};

class ScanBoard : public ROSThreadImage
{
    public:
        ScanBoard(std::string limb);
        ~ScanBoard();

        std::vector<geometry_msgs::Point> getOffsets();

    protected:

        /*
         * scan the board 
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;
        std::vector<geometry_msgs::Point> _offsets;

        /*
         * hover arm above board
         * 
         * @param      N/A
         * return     N/A
         */
        void hoverAboveBoard();

        /*
         * scan the board and calculate cell offsets
         * 
         * @param      N/A
         * return     N/A
         */
        void scan();

        /*
         * move arm downwards until collision w/ a surface; calculate
         * distance between surface and arm starting point
         * 
         * @param      float * dist indicating the distance btw. the surface
         *            and the arm's starting point
         * return     N/A
         */
        void setDepth(float *dist);

        /*
         * calculate cell offsets; also prompts user to move board withing 'reachable zone'
         * (displayed on screen) if board is out of reach of Baxter's arm 
         * 
         * @param      string mode (test/run) indicating a test (does not exit out of scanning loop)
         *            or an actual run (quits once scanning finished)
         * return     N/A
         */
        void processImage(std::string mode, float dist);

        /*
         * isolates black colored object in raw image
         * 
         * @param      Mat displaying black colored objects in raw image
         * return     N/A
         */
        void isolateBlack(cv::Mat * output);

        /*
         * isolates board boundaries from image 
         * 
         * @param      input Mat, output Mat displaying board boundaries, output Contours
         *            storing board contours, integer indicating the area of the board,
         *            and a vector<cv::Point> of the board's four corners
         * return     N/A
         */
        void isolateBoard(Contours * contours, int * board_area,
                          std::vector<cv::Point> * board_corners, cv::Mat input, cv::Mat * output);

        /*
         * finds cell with the higher centroid
         * 
         * @param      returns true if cell i has a higher centroid than cell j; false otherwise
         * return     N/A
         */
        static bool descendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

        /*
         * calculates offset distance from arm to each board cell
         * 
         * @param      board area, cell contours, output Mat displaying cells, and height
         *            from arm to board surface
         * return     N/A
         */
        void setOffsets(int board_area, Contours contours, float dist,
                        cv::Mat *output, std::vector<cv::Point> *centroids);

        /**
         * calculates the perimeter of the area representing all points reachable to the Baxter arm
         * 
         * @param contours       board contours
         * @param dist           distance between starting position and play surface
         * @param board_corners  coordinates of all 4 board corners
         * @param c              input vector containing cell centroids
         * @param cell_to_corner vector representing distance between center of corner cell
         *                       and corner of corner cell
         */
        void setZone(Contours contours, float dist, std::vector<cv::Point> board_corners,
                     std::vector<cv::Point> c, std::vector<cv::Point> * cell_to_corner);
       
        /*
         * checks if Baxter's arm has a joint angles solution for all the calculated cell offsets
         * 
         * @param      N/A
         * return     true if offsets are all reachable; false otherwise
         */
        bool offsetsReachable();

        /*
         * checks if Baxter's arm has a joint angles solution for a certain point on the board
         * scanning image
         * 
         * @param      N/A
         * return     true if point is reachable; false otherwise
         */
        bool pointReachable(cv::Point centroid, float dist);
};

class PutDownToken : public ROSThreadImage, public Gripper
{
    public:
        PutDownToken(std::string limb);     
        ~PutDownToken();

        void setCell(int cell) {_cell = cell;};
        void setOffsets(std::vector<geometry_msgs::Point> offsets) {_offsets = offsets;};

    protected:

        /*
         * puts down token in specified cell
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();

    private:
        int _cell;
        std::vector<geometry_msgs::Point> _offsets;

        /*
         * hover arm above specified cell
         * 
         * @param      N/A
         * return     N/A
         */
        void hoverAboveCell();

        /*
         * hover arm above the board
         * 
         * @param      N/A
         * return     N/A
         */
        void hoverAboveBoard();
};

class TTTController
{
    private:
        MoveToRest   *_rest_class;
        PickUpToken  *_pick_class;
        ScanBoard    *_scan_class;
        PutDownToken  *_put_class; 

    public:
        TTTController(std::string limb);
        ~TTTController();

        /*
         * get most recent state change
         * 
         * @param      N/A
         * return     integer indicating arm's latest state
         */
        int getState();

        void moveToRest();

        void pickUpToken();

        void scanBoard();

        void putDownToken(int cell);     
};

#endif
