/**
 * Copyright (C) 2017 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2.1 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#ifndef __THREAD_SAFE_H__
#define __THREAD_SAFE_H__

#include <mutex>

/**
 * Template class that wraps a <DataType> object and implements
 * a set() and get() functions that are thread safe and mutex
 * protected. This is to avoid having to create a mutex for each
 * data that needs to have protected read/write.
 */
template<class DataType> class ThreadSafe
{
private:
    // The data to protect
    DataType    data;

    // The mutex that protects the data.
    std::mutex mutex;

public:
    /**
     * Constructors
     */
    ThreadSafe() {};
    ThreadSafe(const DataType& _data) : data(_data) {};

    /**
     * Get function to return the data stored in the class
     * @return the data stored in the class
     */
    DataType get()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return data;
    };

    /**
     * Set function to set new data in the object
     * @param  _data the new data
     * @return       true/false if success/failure
     *                          (for now, always true is returned)
     */
    bool set(const DataType& _data)
    {
        std::lock_guard<std::mutex> lock(mutex);
        data = _data;

        return true;
    };

    ~ThreadSafe() {};
};

#endif
