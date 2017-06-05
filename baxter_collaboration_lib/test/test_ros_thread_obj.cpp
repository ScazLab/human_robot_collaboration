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
    int var;                                    // integer to be modified by different threads
    std::vector<ROSThreadObj> threads;          // a vector of ROSThreadObj to accept thread entry functions
    std::vector<void *(*)(void *)> functions;   // a vector of thread entry functions
    std::mutex mtx;                             // mutex for controlled thread access

public:
    /**
     * constructors
     * @param var_value: value of var
     * @param threads_no: number of threads in the threads vector
     * the default constructor creates an object with var_value==0 and threads_no==0
     * both these arguments can be specified using the custom constructor
     */
    ROSThreadObjTest()
    {
        var = 0;
    }
    ROSThreadObjTest(int var_value, int threads_no)
    {
        var = var_value;
        for(int i=0; i < threads_no; i++)
        {
            threads.push_back(ROSThreadObj());
        }
    }

    /**
     * pushes a specified number (one by default) of threads onto the threads vector
     * @param threads_no: number of threads to add
     */
    void add_thread(int threads_no=1)
    {
        for(int i=0; i < threads_no; i++)
        {
            threads.push_back(ROSThreadObj());
        }
    }

    /**
     * does an arithmetic operation on the value of var thread-safely
     */
    bool safe_arithmetic(const std::string& operation, int arg)
    {
        std::lock_guard<std::mutex> lk(mtx);
        if(operation=="addition")
        {
            var = var + arg;
        }
        else if(operation=="subtraction")
        {
            var = var - arg;
        }
        else if(operation=="division")
        {
            var = var / arg; // careful: truncation
        }
        else if(operation=="multiplication")
        {
            var = var * arg;
        }
        else
        {
            return false; // no valid operation
        }
        return true;
    }

    /**
    * @return: gets the value of var thread-safely
    */
    int get_var()
    {
        std::lock_guard<std::mutex> lk(mtx);
        return var;
    }

    /**
     * starts each ROSThreadObj thread with matching thread entry function
     * then joins all threads to the main thread in order
     * @return: true if success, false if failure
     */
    bool start_threads()
    {
        // condition to check each thread can be matched to a thread entry function
        if(threads.size() != functions.size())
        {
            return false;
        }

        // each thread is started with a corresponding thread entry function
        for(int i=0, size=threads.size(); i < size; i++)
        {
            // passing this as an argument binds the thread function to an object instance
            threads[i].start(functions[i], this);
        }

        return true;
    }

    /**
     * joins each ROSThreadObj thread to the main thread in order
     */
    void join_threads()
    {
        for(int i=0, size=threads.size(); i < size; i++)
        {
            threads[i].join();
        }
    }

    /**
     * kills all running threads
     * @return: true if success, false if failure
     */
    bool kill_threads()
    {
        for(int i=0, size=threads.size(); i < size; i++)
        {
            threads[i].kill();
        }

        return true;
    }

    /**
     * thread entry function that prints the value of var
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
     * thread entry function that adds 10 to the value of var in 10 milliseconds
     */
    static void* ten_adder_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->ten_adder();
        return NULL;
    }
    void ten_adder()
    {
        for(int i = 0; i < 10; i++)
        {
            safe_arithmetic("addition", 1);
            sleep(0.1);
        }
    }

    /**
     * thread entry function that multiplies the value of var by 10 in 20 milliseconds
     */
    static void* ten_multiplier_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->ten_multiplier();
        return NULL;
    }
    void ten_multiplier()
    {
        int multiplier = 0;
        for(int i = 0; i < 10; i++)
        {
            multiplier = multiplier + 1;
            sleep(0.2);
        }
        safe_arithmetic("multiplication", multiplier);
    }

    /**
     * thread entry function that divides the value of var by 10 in 30 milliseconds
     */
    static void* ten_divider_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->ten_divider();
        return NULL;
    }
    void ten_divider()
    {
        int multiplier = 0;
        for(int i = 0; i < 10; i++)
        {
            multiplier = multiplier + 1;
            sleep(0.3);
        }
        safe_arithmetic("division", multiplier);
    }

    /**
     * thread entry function that adds 100 to the value of var in 10 seconds
     */
    static void* slow_hundred_adder_wrapper(void* obj)
    {
        ((ROSThreadObjTest*) obj)->slow_hundred_adder();
        return NULL;
    }
    void slow_hundred_adder()
    {
        for(int i = 0; i < 100; i++)
        {
            safe_arithmetic("addition", 1);
            sleep(0.1);
        }
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
        return true;
    }
};

TEST(ROSThreadObjTest, testSingleThread)
{
    ROSThreadObjTest rtot(10, 0);       // create a test object with var==10 and threads_no==0
    rtot.add_function("ten_adder");     // push ten_adder onto functions vector
    rtot.add_thread();                  // push a ROSThreadObj onto threads vector
    rtot.start_threads();               // start the thread with matching function
    rtot.join_threads();                // join the thread to the main thread
    EXPECT_EQ(20, rtot.get_var());    // check final value of var
}

TEST(ROSThreadObjTest, testConcurrentThreads)
{
    ROSThreadObjTest rtot(0, 2);
    rtot.add_function("ten_adder");
    rtot.add_function("ten_adder");
    rtot.start_threads();
    rtot.join_threads();
    EXPECT_EQ(20, rtot.get_var());

    ROSThreadObjTest rtot2(100, 4);
    rtot2.add_function("ten_multiplier");
    rtot2.add_function("ten_divider");
    rtot2.add_function("ten_divider");
    rtot2.add_function("ten_multiplier");
    rtot2.start_threads();
    rtot2.join_threads();
    EXPECT_EQ(100, rtot2.get_var());

    ROSThreadObjTest rtot3(-50, 3);
    rtot3.add_function("slow_hundred_adder");
    rtot3.add_function("slow_hundred_adder");
    rtot3.add_function("slow_hundred_adder");
    rtot3.start_threads();
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
