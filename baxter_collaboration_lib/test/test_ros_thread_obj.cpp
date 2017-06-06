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
    int var;                                 // variable to be modified by different threads
    std::mutex mtx;                          // mutex for controlled thread access

    std::vector<ROSThreadObj>      threads;  // vector of ROSThreadObj
    std::vector<void *(*)(void *)>   funcs;  // vector of thread entry functions

    /**
     * Does an arithmetic operation on the value of var thread-safely
     */
    bool safeArithmetic(const std::string& operation, int arg)
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
     * Adds 10 to the value of var in 10 milliseconds
     */
    static void* tenAdderWrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->tenAdder();
        return NULL;
    }

    void tenAdder()
    {
        for (size_t i = 0; i < 10; ++i)
        {
            safeArithmetic("addition", 1);
            sleep(0.01);
        }
    }

    /**
     * Multiplies the value of var by 10 in 20 milliseconds
     */
    static void* tenMultiplierWrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->tenMultiplier();
        return NULL;
    }

    void tenMultiplier()
    {
        int multiplier = 0;

        for (size_t i = 0; i < 10; ++i)
        {
            multiplier = multiplier + 1;
            sleep(0.02);
        }

        safeArithmetic("multiplication", multiplier);
    }

    /**
     * Divides the value of var by 10 in 30 milliseconds
     */
    static void* tenDividerWrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->tenDivider();
        return NULL;
    }

    void tenDivider()
    {
        int divider = 0;

        for (size_t i = 0; i < 10; ++i)
        {
            divider = divider + 1;
            sleep(0.03);
        }

        safeArithmetic("division", divider);
    }

    /**
     * adds 100 to the value of var in 10 seconds
     */
    static void* slowHundredAdderWrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->slowHundredAdder();
        return NULL;
    }

    void slowHundredAdder()
    {
        for (size_t i = 0; i < 100; ++i)
        {
            safeArithmetic("addition", 1);
            sleep(0.01);
        }
    }

public:

    /**
     * Class constructor
     *
     * @param _var: value of var (default 0)
     */
    explicit ROSThreadObjTest(int _var = 0) : var(_var) {}

    /**
     * Gets the value of var thread-safely
     *
     * @return the value of var
     */
    int getVar()
    {
        std::lock_guard<std::mutex> lk(mtx);
        return var;
    }

    /**
    * pushes a specified function onto the functions vector
    * this is done indirectly by pushing the thread function wrapper
    *
    * @param name pointer to a thread entry function
    */
    bool addFunction(const std::string& _name)
    {
        if     (_name ==         "tenAdder")
        {
            funcs.push_back(tenAdderWrapper);
        }
        else if(_name ==    "tenMultiplier")
        {
            funcs.push_back(tenMultiplierWrapper);
        }
        else if(_name ==       "tenDivider")
        {
            funcs.push_back(tenDividerWrapper);
        }
        else if(_name == "slowHundredAdder")
        {
            funcs.push_back(slowHundredAdderWrapper);
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
    bool startThreads()
    {
        // condition to check each thread can be matched to a thread entry function
        if(threads.size() != funcs.size())
        {
            return false;
        }

        // each thread is started with a corresponding thread entry function
        for (size_t i=0; i < threads.size(); ++i)
        {
            // passing this as an argument binds the thread function to an object instance
            threads[i].start(funcs[i], this);
        }

        return true;
    }

    /**
     * Joins each ROSThreadObj thread to the main thread in order
     */
    bool joinThreads()
    {
        for (size_t i=0; i < threads.size(); ++i)
        {
            threads[i].join();
        }

        return true;
    }

    /**
     * Kills all running threads
     *
     * @return: true/false if success/failure
     */
    bool killThreads()
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
    ROSThreadObjTest rtot(10);                  // create test object with var==10
    EXPECT_TRUE(rtot.addFunction("tenAdder"));  // push tenAdder onto functions vector
    EXPECT_TRUE(rtot.startThreads());           // start the thread with matching function
    EXPECT_TRUE(rtot.joinThreads());            // join the thread to the main thread
    EXPECT_EQ(20, rtot.getVar());               // check final value of var
}

TEST(ROSThreadObjTest, testConcurrentThreads)
{
    ROSThreadObjTest rtot(0);
    EXPECT_TRUE(rtot.addFunction("tenAdder"));
    EXPECT_TRUE(rtot.addFunction("tenAdder"));
    EXPECT_TRUE(rtot.startThreads());
    EXPECT_TRUE(rtot.joinThreads());
    EXPECT_EQ(20, rtot.getVar());

    ROSThreadObjTest rtot2(100);
    EXPECT_TRUE(rtot2.addFunction("tenMultiplier"));
    EXPECT_TRUE(rtot2.addFunction("tenDivider"));
    EXPECT_TRUE(rtot2.addFunction("tenDivider"));
    EXPECT_TRUE(rtot2.addFunction("tenMultiplier"));
    EXPECT_TRUE(rtot2.startThreads());
    EXPECT_TRUE(rtot2.joinThreads());
    EXPECT_EQ(100, rtot2.getVar());

    ROSThreadObjTest rtot3(-50);
    EXPECT_TRUE(rtot3.addFunction("slowHundredAdder"));
    EXPECT_TRUE(rtot3.addFunction("slowHundredAdder"));
    EXPECT_TRUE(rtot3.addFunction("slowHundredAdder"));
    EXPECT_TRUE(rtot3.startThreads());
    EXPECT_TRUE(rtot3.joinThreads());
    EXPECT_EQ(250, rtot3.getVar());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ros_thread_obj");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
