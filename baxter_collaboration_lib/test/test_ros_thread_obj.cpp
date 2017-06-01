#include <gtest/gtest.h>
#include <memory>

#include "robot_utils/ros_thread_obj.h"

#include <mutex>

using namespace std;

// pthreads prevent use of shared_ptr. So for now, mutex is needed for concurrent threads
std::mutex m;

/**
 * the purpose of this class is to provide an object that can contain multiple ROSThreadObj
 * as well as thread functions, and observe their combined effect on a shared variable
 */
class ROSThreadObjTest
{
private:
    std::shared_ptr<int> var;                   // integer to be modified by different threads
    std::vector<ROSThreadObj> threads;          // a vector of ROSThreadObj to accept thread entry functions
    std::vector<void *(*)(void *)> functions;   // a vector of thread entry functions

public:
    /**
     * constructors
     * @param var_value: value of shared_ptr var
     * @param threads_no: number of threads in the threads vector
     * the default constructor creates an object with var_value==0 and threads_no==0
     * both these arguments can be specified using the custom constructor
     */
    ROSThreadObjTest(): var(std::make_shared<int>(0)) {}
    ROSThreadObjTest(int var_value, int threads_no): var(std::make_shared<int>(var_value))
    {
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
     * pushes a specified function onto the functions vector
     * @param func: the pointer to a thread entry function
     */
    void add_function(void *(* func)(void *)) // accepts functions with void* input and output
    {
        functions.push_back(func);
    }

    /**
    * @return: the value pointed to by the shared_ptr var
    */
    int var_value()
    {
        return *var;
    }

    /**
    * @return: the reference/use count of the shared_ptr
    */
    int var_count()
    {
        return var.use_count();
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
            // the raw_ptr is retrieved from the shared_ptr for casting
            // (according to pthread specs) and sent to the thread entry function
            threads[i].start(functions[i], static_cast<void*>(var.get()));
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
     * Provides useful output for debugging
     */
    void test_info()
    {
        cout << "This ROSThreadObjTest object's address is: " << this << endl;
        cout << "### VAR INFO ###" << endl;
        cout << "*var is: " << *var << endl;
        cout << "&var is: " << &var << endl;
        cout << "var is: " << var << endl;
        cout << "var.get() is: " << var.get() << endl;
        cout << "### THREADS INFO ###" << endl;
        cout << "The threads vector size is: " << threads.size() << endl;
        cout << "The threads vector address is: " << &threads << endl;
        cout << "threads[0] has a type: " << typeid(threads[0]).name() << endl;
        cout << "### FUNCTIONS INFO ###" << endl;
        cout << "The functions vector size is: " << functions.size() << endl;
        cout << "functions[0] has a type: " << typeid(functions[0]).name() << endl;
    }
};

/**
 * thread entry function that prints the value of var
 * @param var: the value pointed to by shared_ptr var
 */
void* print_var(void* var)
{
    auto var_instance = static_cast<int*>(var);
    m.lock();
    cout << *var_instance << endl;
    m.unlock();
    return (void*) 1;
}

/**
 * thread entry function that adds 10 to the value of var in 10 milliseconds
 * @param var: the value pointed to by shared_ptr var
 */
void* ten_adder(void* var)
{
    auto var_instance = static_cast<int*>(var);
    m.lock();
    for(int i = 0; i < 10; i++)
    {
        *var_instance = *var_instance + 1;
        sleep(0.1);
    }
    m.unlock();
    return (void*) 1;
}

/**
 * thread entry function that multiplies the value of var by 10 in 20 milliseconds
 * @param var: the value pointed to by shared_ptr var
 */
void* ten_multiplier(void* var)
{
    auto var_instance = static_cast<int*>(var);
    int multiplier = 0;
    m.lock();
    for(int i = 0; i < 10; i++)
    {
        multiplier = multiplier + 1;
        sleep(0.2);
    }
    *var_instance = *var_instance * multiplier;
    m.unlock();
    return (void*) 1;
}

/**
 * thread entry function that divides the value of var by 10 in 30 milliseconds
 * @param var: the value pointed to by shared_ptr var
 */
void* ten_divider(void* var)
{
    auto var_instance = static_cast<int*>(var);
    int multiplier = 0;
    m.lock();
    for(int i = 0; i < 10; i++)
    {
        multiplier = multiplier + 1;
        sleep(0.3);
    }
    *var_instance = *var_instance / multiplier; // careful: truncation
    m.unlock();
    return (void*) 1;
}

/**
 * thread entry function that adds 100 to the value of var in 10 seconds
 * @param var: the value pointed to by shared_ptr var
 */
void* slow_hundred_adder(void* var)
{
    auto var_instance = static_cast<int*>(var);
    m.lock();
    for(int i = 0; i < 100; i++)
    {
        *var_instance = *var_instance + 1;
        sleep(0.1);
    }
    m.unlock();
    return (void*) 1;
}

TEST(ROSThreadObjTest, testSingleThread)
{
    ROSThreadObjTest rtot(10, 0);       // create a test object with var==10 and threads_no==0
    rtot.add_function(ten_adder);       // push ten_adder onto functions vector
    rtot.add_thread();                  // push a ROSThreadObj onto threads vector
    rtot.start_threads();               // start the thread with matching function
    rtot.join_threads();                // join the thread to the main thread
    EXPECT_EQ(20, rtot.var_value());    // check final value of var
}

TEST(ROSThreadObjTest, testConcurrentThreads)
{
    ROSThreadObjTest rtot(0, 2);
    rtot.add_function(ten_adder);
    rtot.add_function(ten_adder);
    rtot.start_threads();
    rtot.join_threads();
    EXPECT_EQ(20, rtot.var_value());

    ROSThreadObjTest rtot2(100, 4);
    rtot2.add_function(ten_multiplier);
    rtot2.add_function(ten_divider);
    rtot2.add_function(ten_divider);
    rtot2.add_function(ten_multiplier);
    rtot2.start_threads();
    rtot2.join_threads();
    EXPECT_EQ(100, rtot2.var_value());

    ROSThreadObjTest rtot3(-50, 3);
    rtot3.add_function(slow_hundred_adder);
    rtot3.add_function(slow_hundred_adder);
    rtot3.add_function(slow_hundred_adder);
    rtot3.start_threads();
    rtot3.join_threads();
    EXPECT_EQ(250, rtot3.var_value());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ros_thread_obj");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
