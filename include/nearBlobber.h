/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  Tanis Mar
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __NEARBLOBBER_H__
#define __NEARBLOBBER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <time.h>
#include <string>
#include <iostream>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

class NearBlobber : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >
{
private:

    std::string moduleName;                     // string containing module name
    // std::string worldInPortName;               // string containing world in info port name
    std::string dispInPortName;                 // string containing disparity image port name
    std::string targetOutPortName;              // string containing output target port name
    std::string imageOutPortName;               // string containing output image port name
    std::string imgBinOutPortName;               // string containing output image port name
    
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >	dispInPort;		// Receives disparity greyscale image --- Handled by the clas itself    

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >	imageOutPort;	// output image Port with info drawn over
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >	imgBinOutPort;	// output binary image of closest blob
    yarp::os::BufferedPort<yarp::os::Bottle>							targetOutPort;	// Send coordinates of closest point.      
    
    /* Pointer to the Resource Finder */
    yarp::os::ResourceFinder *moduleRF;

    /* Algorithm Variables */
    int backgroundThresh;
    int frontThresh;
    int cannyThresh;
    int minBlobSize;
    bool verbose;
    
    int dispThreshRatioLow;
    int dispThreshRatioHigh;
    
    int gaussSize;
    int margin;

protected:
   
public:
    /**
     * constructor
     * @param moduleName is passed to the thread in order to initialise all the ports correctly
     */
    NearBlobber( const std::string &moduleName, yarp::os::ResourceFinder &module );
    ~NearBlobber();

    bool setThresh(int low, int high);
    bool setVerbose(std::string verb);
    bool setMargin(int mrg);

    bool        open();
    void        close();
    void        onRead( yarp::sig::ImageOf<yarp::sig::PixelBgr> &img );
    void        interrupt();
    
    yarp::os::Semaphore         mutex;          //semaphore for accessing/modifying within the callback
       
};

class NearBlobberModule:public yarp::os::RFModule
{
    /* module parameters */
    std::string             moduleName;
    std::string             handlerPortName;    
    yarp::os::RpcServer     rpcInPort;

    /* pointer to a new thread */
    NearBlobber             *detector;
    bool                    closing;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module

    double getPeriod();
    bool updateModule();
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};


#endif
//empty line to make gcc happy
