/*
 * Copyright (C) 2014 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  tanis.mar@iit.it
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

#include "nearBlobber.h"

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/**********************************************************/
bool NearBlobberModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("nearBlobber"), "module name (string)").asString();

    setName(moduleName.c_str());

    handlerPortName =  "/";
    handlerPortName += getName();
    handlerPortName +=  "/rpc:i";

    if (!rpcInPort.open(handlerPortName.c_str()))    {
        fprintf(stdout, "%s : Unable to open input RPC port %s\n", getName().c_str(), handlerPortName.c_str());
        return false;
    }

    attach(rpcInPort);
    closing = false;

    /* create the thread and pass pointers to the module parameters */
    detector = new NearBlobber( moduleName, rf );

    /* now start the thread to do the work */
    detector->open();
    return true ;
}

/**********************************************************/
bool NearBlobberModule::interruptModule()
{
    closing = true;
    rpcInPort.interrupt();
    detector->interrupt();
    return true;
}

/**********************************************************/
bool NearBlobberModule::close()
{
    rpcInPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");   
    detector->close();
    fprintf(stdout, "deleting thread\n");
    delete detector;
    fprintf(stdout, "done deleting thread\n");
    return true;
}

/**********************************************************/
bool NearBlobberModule::updateModule()
{
    return !closing;
}

/**********************************************************/
bool NearBlobberModule::respond(const Bottle &command, Bottle &reply)
{
    /* This method is called when a command string is sent via RPC */
    reply.clear();  // Clear reply bottle

    /* Get command string */
    string receivedCmd = command.get(0).asString().c_str();
    int responseCode;   //Will contain Vocab-encoded response
    if (receivedCmd == "thresh"){
        bool ok = detector->setThresh(command.get(1).asInt(), command.get(2).asInt());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Threshold for disparity considered set. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "verbose"){
        bool ok = detector->setVerbose(command.get(1).asString());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "help"){
        reply.addVocab(Vocab::encode("many"));
        responseCode = Vocab::encode("ack");
        reply.addString("Available commands are:");
        reply.addString("margin (int) - sets the margin (in pixels) that the ROI keeps around the closest blob.");
        reply.addString("thresh (int) (int)- sets higher and lower limit of disparity in terms of luminosity (0-255) that is considered. In other words, objects with luminosity outside boundaries wont be considered.");
        reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
        reply.addString("help - produces this help.");
        reply.addString("quit - closes the module.");
        
        reply.addVocab(responseCode);
        return true;
    } else if (receivedCmd == "quit"){
        responseCode = Vocab::encode("ack");
        reply.addVocab(responseCode);
        closing = true;
        return true;
    }
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;
}

/**********************************************************/
double NearBlobberModule::getPeriod()
{
    return 0.1;
}

/**********************************************************/
NearBlobber::~NearBlobber()
{
    
}

/**********************************************************/
NearBlobber::NearBlobber( const string &moduleName, ResourceFinder &rf)
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
    this->moduleRF = &rf;
}

/**********************************************************/
bool NearBlobber::open()
{
    this->useCallback();
    fprintf(stdout,"Parsing parameters\n");	

    verbose = moduleRF->check("verbose", Value(false)).asBool();
    margin = moduleRF->check("margin", Value(20)).asInt();
    backgroundThresh = moduleRF->check("backgroundThresh", Value(50)).asInt();		// threshold of intensity if the disparity image, under which info is ignored.
    frontThresh = moduleRF->check("frontThresh", Value(190)).asInt();		// threshold of intensity if the disparity image, above which info is ignored.
    cannyThresh = moduleRF->check("cannyThresh", Value(20)).asDouble();
    minBlobSize = moduleRF->check("minBlobSize", Value(400)).asInt();
    gaussSize = moduleRF->check("gaussSize", Value(5)).asInt();
    dispThreshRatioLow = moduleRF->check("dispThreshRatioLow", Value(10)).asInt();    
    dispThreshRatioHigh = moduleRF->check("dispThreshRatioHigh", Value(20)).asInt();


    bool ret=true;
    //create all ports
    fprintf(stdout,"Opening ports\n");

    /* Inputs ports */

    dispInPortName = "/" + moduleName + "/disp:i";
    BufferedPort<ImageOf<PixelBgr>  >::open( dispInPortName.c_str() );    

    /* Output ports */

    imageOutPortName = "/" + moduleName + "/img:o";
    imageOutPort.open(imageOutPortName);

    imgBinOutPortName = "/" + moduleName + "/imgBin:o";
    imgBinOutPort.open(imgBinOutPortName);
    
    targetOutPortName = "/" + moduleName + "/target:o";
    targetOutPort.open(targetOutPortName);

    return ret;
}

/**********************************************************/
void NearBlobber::close()
{
    fprintf(stdout,"now closing ports...\n");
    
    BufferedPort<ImageOf<PixelBgr>  >::close();

    imageOutPort.close();
    imgBinOutPort.close();
    targetOutPort.close();
            
    fprintf(stdout,"finished closing input and output ports...\n");
}

/**********************************************************/
void NearBlobber::interrupt()
{	
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
   
    BufferedPort<ImageOf<PixelBgr>  >::interrupt();

    imageOutPort.interrupt();
    imgBinOutPort.interrupt();
    targetOutPort.interrupt();
    
    fprintf(stdout,"finished interrupt ports\n");
}

bool NearBlobber::setThresh(int low, int high)
{
    if ((low<0) ||(low>255)||(high<0) ||(high>255)) {
        fprintf(stdout,"Please select valid luminance values (0-255). \n");
        return false;
    }
    fprintf(stdout,"New Threshold is : %i, %i\n", low, high);
    this->backgroundThresh = low;
    this->frontThresh = high;
    return true;
}

bool NearBlobber::setVerbose(string verb)
{
    if (verb == "ON"){
        verbose = true;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    } else if (verb == "OFF"){
        verbose = false;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    }    
    return false;
}

bool NearBlobber::setMargin(int mrg)
{
    fprintf(stdout,"New margin : %f\n", mrg);
    this->margin = mrg;
    return true;
}


/**********************************************************/
void NearBlobber::onRead(ImageOf<PixelBgr> &disparity)
{
    mutex.wait();    
    if(verbose){
        cout << endl;
        cout << "================ LOOP =================== "<< endl;}
    Scalar blue = Scalar(255,0,0);
    Scalar green = Scalar(0,255,0);
    Scalar red = Scalar(0,0,255);
    Scalar white = Scalar(255,255,255);

	/* Format disparty data to Mat grayscale */
    Mat disp((IplImage*) disparity.getIplImage());			
    cvtColor(disp, disp, CV_BGR2GRAY);						// Brg to grayscale

    /* Prepare output image for visualization */
    ImageOf<PixelBgr> &imageOut  = imageOutPort.prepare();
    imageOut = disparity;	
    Mat imOut((IplImage*)imageOut.getIplImage(),false);

    /* Prepare binary image to ouput closesrt blob */
	ImageOf<PixelMono> &imgBin = imgBinOutPort.prepare();		// prepare an output image
	imgBin.resize(disparity.width(), disparity.height());		// Initialize features image
    imgBin.zero();
	Mat imgBinMat((IplImage*)imgBin.getIplImage(),false);
    
    /* Prepare output target port */
    Bottle &target = targetOutPort.prepare();
    target.clear();

    /* Filter disparity image to reduce noise */
    GaussianBlur(disp, disp, Size(gaussSize,gaussSize), 1.5, 1.5);
        //erode(disp,disp,Mat());
    //dilate(disp,disp, Mat());
    Mat threshIm;
    threshold(disp, threshIm, backgroundThresh, 1, CV_THRESH_BINARY);			// First
    multiply(disp, threshIm, disp);	
    dilate(disp, disp,Mat(), Point(-1,-1), 4);
    GaussianBlur(disp, disp, Size(gaussSize,gaussSize), 2, 2); 
    erode(disp, disp, Mat(), Point(-1,-1), 2);    
    

    cvtColor(disp, imOut, CV_GRAY2BGR);						// Grayscale to BGR    
    
    /* Find closest valid blob */
    double minVal, maxVal; 
    Point minLoc, maxLoc;	
    int fillFlags = 8 + FLOODFILL_FIXED_RANGE;// Set the flags for floodfill

    int fillSize = 0;	
    Mat aux = disp.clone();
    Mat fillMask = Mat::zeros(disp.rows + 2, disp.cols + 2, CV_8U);
    while (fillSize < minBlobSize){			
        minMaxLoc( aux, &minVal, &maxVal, &minLoc, &maxLoc );		// Look for brighter (closest) point
        fillSize = floodFill(aux, maxLoc, 0, 0, Scalar(maxVal/dispThreshRatioLow), Scalar(maxVal/dispThreshRatioHigh), fillFlags);	// If its too small, paint it black and search again
    }

    floodFill(disp, fillMask, maxLoc, 255, 0, Scalar(maxVal/dispThreshRatioLow), Scalar(maxVal/dispThreshRatioHigh), FLOODFILL_MASK_ONLY + fillFlags);	// Paint closest valid blob white
    //threshold(disp, disp, 250, 255, CV_THRESH_BINARY);
    
    /* Find Contours */
    Mat edges;	
    vector<vector<Point > > contours;
    vector<Vec4i> hierarchy;
    //Canny( disp, edges, cannyThresh, cannyThresh*3, 3 );			// Detect edges using canny	
    findContours( fillMask(Range(1,disp.rows),Range(1,disp.cols)), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
    /* If any blob is found */
    if (contours.size()>0){
        /* Double check that only the bigger blob is selected as the valid one*/
        int blobI = 0;
        for( int c = 0; c < contours.size(); c++ ){
			double a = contourArea(contours[c]);	    					// Find the area of contour
			if(a > minBlobSize){											// Keep only the bigger
                blobI = c;
            }
        }
        /* Mark closest blob for visualization*/        
        drawContours(imOut, contours, blobI, white, 2, 8); 
        drawContours(imgBinMat, contours, blobI, white, -1, 8); 

        /* check and plt the ROI */
        Rect blobBox = boundingRect(contours[blobI]);
        if(verbose)
            {cout << " min enclosing Box is  ["<< blobBox.tl().x << "," << blobBox.tl().y << "]:["<<blobBox.br().x << ","<<blobBox.br().y << "]"<< endl;}
        Rect imBox(Point(blobBox.tl().x-margin,blobBox.tl().y-margin),Point(blobBox.br().x+margin,blobBox.br().y+margin));
        rectangle(imOut, imBox, blue, 2 );

        /* Get and draw centroid of the closest blob */ 
        Point center2DCoords;
        Moments mu = moments( contours[blobI], false );		
        center2DCoords = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( imOut, center2DCoords, 4, red, -1, 8, 0 );
        
        /* Compute the average distance of each detected blob */ 
        Mat reachness(disp.size(), CV_8UC3, Scalar(0,0,0));
        drawContours( reachness, contours, -1, green, CV_FILLED, 8);		// Paint Mat reachness(disp.size(), CV_8UC3, Scalar(0,0,0)); far blobs
        addWeighted( imOut, 0.7, reachness, 0.3, 0.0, imOut);

        target.addDouble(imBox.tl().x);
        target.addDouble(imBox.tl().y);
        target.addDouble(imBox.br().x);
        target.addDouble(imBox.br().y);      
    }
    
    mutex.post();

    /* write info on output ports */
    imageOutPort.write();
    imgBinOutPort.write();
    targetOutPort.write();

}
//empty line to make gcc happy

