#include <gtest/gtest.h>
#include <memory>
#include <mutex>

#include "robot_utils/ros_thread_obj.h"

using namespace std;

/**
 * the purpose of this class is to provide an object that can contain multiple ROSThreadObj
 * as well as thread functions, and observe their combined effect on a shared variable
 */
class ROSThreadObjTest
{

private:
    int var;                                    // variable to be modified by different threads
    std::mutex mtx;                             // mutex for controlled thread access

    std::vector<ROSThreadObj>        threads;   // vector of ROSThreadObj
    std::vector<void *(*)(void *)> functions;   // vector of thread entry functions

    /**
     * Does an arithmetic operation on the value of var thread-safely
     */
    bool safe_arithmetic(const std::string& operation, int arg)
    {
        std::lock_guard<std::mutex> lk(mtx);

        if     (operation=="addition")       {   var += arg; }
        else if(operation=="subtraction")    {   var -= arg; }
        else if(operation=="division")       {   var /= arg; }
        else if(operation=="multiplication") {   var *= arg; }
        else                                 { return false; }

        return true;
    }

    /**
     * Prints the value of var
     */
    static void* print_var_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->print_var();

        return NULL;
    }

    void print_var()
    {
        cout << get_var() << endl;
    }

    /**
     * Adds 10 to the value of var in 10 milliseconds
     */
    static void* ten_adder_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->ten_adder();
        return NULL;
    }

    void ten_adder()
    {
        for (size_t i = 0; i < 10; ++i)
        {
            safe_arithmetic("addition", 1);
            sleep(0.01);
        }
    }

    /**
     * Multiplies the value of var by 10 in 20 milliseconds
     */
    static void* ten_multiplier_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->ten_multiplier();
        return NULL;
    }

    void ten_multiplier()
    {
        int multiplier = 0;

        for (size_t i = 0; i < 10; ++i)
        {
            multiplier = multiplier + 1;
            sleep(0.02);
        }

        safe_arithmetic("multiplication", multiplier);
    }

    /**
     * Divides the value of var by 10 in 30 milliseconds
     */
    static void* ten_divider_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->ten_divider();

        return NULL;
    }

    void ten_divider()
    {
        int divider = 0;

        for (size_t i = 0; i < 10; ++i)
        {
            divider = divider + 1;
            sleep(0.03);
        }

        safe_arithmetic("division", divider);
    }

    /**
     * adds 100 to the value of var in 10 seconds
     */
    static void* slow_hundred_adder_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->slow_hundred_adder();
        return NULL;
    }

    void slow_hundred_adder()
    {
        for (size_t i = 0; i < 100; ++i)
        {
            safe_arithmetic("addition", 1);
            sleep(0.01);
        }
    }

public:

    /**
     * Class constructor
     *
     * @param _var: value of var (default 0)
     */
    ROSThreadObjTest(int _var = 0) : var(_var) {}

    /**
     * Gets the value of var thread-safely
     *
     * @return the value of var
     */
    int get_var()
    {
        std::lock_guard<std::mutex> lk(mtx);
        return var;
    }

    /**
    * pushes a specified function onto the functions vector
    * this is done indirectly by pushing the thread function wrapper
    * @param func: the pointer to a thread entry function
    */
    bool add_function(const std::string& func_name)
    {
        if(func_name == "print_var")
        {
            functions.push_back(print_var_wrapper);
        }
        else if(func_name == "ten_adder")
        {
            functions.push_back(ten_adder_wrapper);
        }
        else if(func_name == "ten_multiplier")
        {
            functions.push_back(ten_multiplier_wrapper);
        }
        else if(func_name == "ten_divider")
        {
            functions.push_back(ten_divider_wrapper);
        }
        else if(func_name == "slow_hundred_adder")
        {
            functions.push_back(slow_hundred_adder_wrapper);
        }
        else
        {
            return false;
        }

        threads.push_back(ROSThreadObj());

        return true;
    }

    /**
     * Starts each ROSThreadObj thread with matching thread entry function,
     * then joins all threads to the main thread in order
     *
     * @return: true/false if success/failure
     */
    bool start_threads()
    {
        // condition to check each thread can be matched to a thread entry function
        if(threads.size() != functions.size())
        {
            return false;
        }

        // each thread is started with a corresponding thread entry function
        for (size_t i=0; i < threads.size(); ++i)
        {
            // passing this as an argument binds the thread function to an object instance
            threads[i].start(functions[i], this);
        }

        return true;
    }

    /**
     * Joins each ROSThreadObj thread to the main thread in order
     */
    void join_threads()
    {
        for (size_t i=0; i < threads.size(); ++i)
        {
            threads[i].join();
        }
    }

    /**
     * Kills all running threads
     *
     * @return: true/false if success/failure
     */
    bool kill_threads()
    {
        for (size_t i=0; i < threads.size(); ++i)
        {
            threads[i].kill();
        }

        return true;
    }
};

TEST(ROSThreadObjTest, testSingleThread)
{
    ROSThreadObjTest rtot(10);                    // create test object with var==10
    EXPECT_TRUE(rtot.add_function("ten_adder"));  // push ten_adder onto functions vector
    EXPECT_TRUE(rtot.start_threads());            // start the thread with matching function
    rtot.join_threads();                          // join the thread to the main thread
    EXPECT_EQ(20, rtot.get_var());                // check final value of var
}

TEST(ROSThreadObjTest, testConcurrentThreads)
{
    ROSThreadObjTest rtot(0);
    EXPECT_TRUE(rtot.add_function("ten_adder"));
    EXPECT_TRUE(rtot.add_function("ten_adder"));
    EXPECT_TRUE(rtot.start_threads());
    rtot.join_threads();
    EXPECT_EQ(20, rtot.get_var());

    ROSThreadObjTest rtot2(100);
    EXPECT_TRUE(rtot2.add_function("ten_multiplier"));
    EXPECT_TRUE(rtot2.add_function("ten_divider"));
    EXPECT_TRUE(rtot2.add_function("ten_divider"));
    EXPECT_TRUE(rtot2.add_function("ten_multiplier"));
    EXPECT_TRUE(rtot2.start_threads());
    rtot2.join_threads();
    EXPECT_EQ(100, rtot2.get_var());

    ROSThreadObjTest rtot3(-50);
    EXPECT_TRUE(rtot3.add_function("slow_hundred_adder"));
    EXPECT_TRUE(rtot3.add_function("slow_hundred_adder"));
    EXPECT_TRUE(rtot3.add_function("slow_hundred_adder"));
    EXPECT_TRUE(rtot3.start_threads());
    rtot3.join_threads();
    EXPECT_EQ(250, rtot3.get_var());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ros_thread_obj");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
