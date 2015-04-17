/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Tanis Mar, Giulia Pasquale
 * email:  tanis.mar@iit.it, giulia.pasquale@iit.it
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

#include "nearBlobberModule.hpp"

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


bool NearBlobberModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("nearBlobber"), "module name (string)").asString();

    setName(moduleName.c_str());

    handlerPortName =  "/";
    handlerPortName += getName();
    handlerPortName +=  "/rpc:i";

    if (!handlerPort.open(handlerPortName.c_str()))    {
        fprintf(stdout, "%s : Unable to open RPC port %s\n", getName().c_str(), handlerPortName.c_str());
        return false;
    }
    attach(handlerPort);

    blobPort = new NearBlobberPort( moduleName, rf );

    blobPort->open();

    closing = false;

    return true ;
}

bool NearBlobberModule::interruptModule()
{
    closing = true;

    handlerPort.interrupt();

    blobPort->interrupt();

    return true;
}

bool NearBlobberModule::close()
{
    handlerPort.close();

    fprintf(stdout, "starting the shutdown procedure\n");   

    blobPort->close();

    fprintf(stdout, "deleting thread\n");
    delete blobPort;
    fprintf(stdout, "done deleting thread\n");

    return true;
}

bool NearBlobberModule::updateModule()
{
    return !closing;
}

bool NearBlobberModule::respond(const Bottle &command, Bottle &reply)
{

    reply.clear();

    /* Get command string */
    string receivedCmd = command.get(0).asString().c_str();

    int responseCode;   // contain Vocab-encoded response

    if (receivedCmd == "margin")
    {
        bool ok = blobPort->setMargin(command.get(1).asInt());
        if (ok)
        	responseCode = Vocab::encode("ack");
            else
            {
                fprintf(stdout,"Margin for ROI cannot be set. \n");
                responseCode = Vocab::encode("nack");
            }
        reply.addVocab(responseCode);
        return true;
    }
    if (receivedCmd == "thresh")
    {
        bool ok = blobPort->setThresh(command.get(1).asInt(), command.get(2).asInt());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Threshold cannot be set. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;
    }
    else if (receivedCmd == "help")
    {
        reply.addVocab(Vocab::encode("many"));

        responseCode = Vocab::encode("ack");

        reply.addString("Available commands are:");
        reply.addString("margin (int) - sets the margin (in pixels) that the ROI keeps around the closest blob.");
        reply.addString("thresh (int) (int)- sets higher and lower luminosity limits (0-255) that are considered. Objects with luminosity outside boundaries wont be considered.");
        reply.addString("help - produces this help.");
        reply.addString("quit - closes the module.");
        
        reply.addVocab(responseCode);
        return true;
    }
    else if (receivedCmd == "quit")
    {
        responseCode = Vocab::encode("ack");

        reply.addVocab(responseCode);

        closing = true;
        return true;
    }
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;
}

double NearBlobberModule::getPeriod()
{
    return 0.1;
}

NearBlobberPort::NearBlobberPort( const string &_moduleName, ResourceFinder &rf)
{

    this->moduleName = _moduleName;

    moduleRF = &rf;

    fprintf(stdout,"Parsing parameters...\n");

    int imH = moduleRF->check("imH", Value(240)).asInt();
    int imW = moduleRF->check("imW", Value(320)).asInt();


    int margin = moduleRF->check("margin", Value(20)).asInt();
    cropSize = 0;

    if (rf.check("cropSize"))
    {
    	Value &vCropSize=rf.find("cropSize");

    	if (!vCropSize.isString())
    	{
    		cropSize = vCropSize.asInt();
    		margin = 0; // not used in this case
    	}
    }

    // threshold of intensity of the image under which info is ignored
    int backgroundThresh = moduleRF->check("backgroundThresh", Value(50)).asInt();
    // threshold of intensity of the image above which info is ignored
    int frontThresh = moduleRF->check("frontThresh", Value(190)).asInt();

    int minBlobSize = moduleRF->check("minBlobSize", Value(400)).asInt();

    int gaussSize = moduleRF->check("gaussSize", Value(5)).asInt();

    int imageThreshRatioLow = moduleRF->check("imageThreshRatioLow", Value(10)).asInt();
    int imageThreshRatioHigh = moduleRF->check("imageThreshRatioHigh", Value(20)).asInt();

    blobExtractor = NULL;

    blobExtractor = new nearBlobber(imH, imW,
    		margin,
    		backgroundThresh, frontThresh,
    		minBlobSize, gaussSize,
    		imageThreshRatioLow, imageThreshRatioHigh);
}

bool NearBlobberPort::open()
{

    this->useCallback();

    fprintf(stdout,"Opening ports...\n");

    /* Inputs */

    imgInPortName = "/" + moduleName + "/img:i";
    BufferedPort<ImageOf<PixelBgr>  >::open( imgInPortName.c_str() );

    /* Outputs */

    optOutPortName = "/" + moduleName + "/opt:o";
    optOutPort.open(optOutPortName);

    blobsOutPortName = "/" + moduleName + "/blobs:o";
    blobsOutPort.open(blobsOutPortName);
    
    cropOutPortName = "/" + moduleName + "/crop:o";
    cropOutPort.open(cropOutPortName);

    return true;
}

void NearBlobberPort::close()
{
    fprintf(stdout,"Closing ports...\n");
    
    optOutPort.close();

    blobsOutPort.close();
    cropOutPort.close();

    BufferedPort<ImageOf<PixelBgr>  >::close();
            
    fprintf(stdout,"Finished closing ports...\n");
}

void NearBlobberPort::interrupt()
{	
   
    fprintf(stdout,"Attempting to interrupt ports...\n");

    optOutPort.interrupt();

    blobsOutPort.interrupt();
    cropOutPort.interrupt();

    BufferedPort<ImageOf<PixelBgr>  >::interrupt();
    
    fprintf(stdout,"Finished interrupting ports...\n");
}

bool NearBlobberPort::setThresh(int low, int high)
{
    return blobExtractor->setThresh(low, high);

}

bool NearBlobberPort::setMargin(int mrg)
{
    return blobExtractor->setMargin(mrg);
}

void NearBlobberPort::onRead(ImageOf<PixelBgr> &input)
{
    mutex.wait();

    /* Get the envelope from the input image */

    Stamp stamp;
    BufferedPort<ImageOf<PixelBgr>  >::getEnvelope(stamp);

    /* Prepare output data structures */

    std::vector<int> centroid;
    std::vector<int> roi;
    cv::Mat blobMat;

    /* Prepare the buffer, call the extractor, clear the buffer */

    imagesMatBuffer.push_back(cv::Mat( (IplImage*)input.getIplImage() ));

    double blobSize = blobExtractor->extractBlob(imagesMatBuffer, roi, centroid, blobMat);

    if (blobSize>0)
    {

    	/* Prepare and write output blobs port */

    	Bottle blobsBottle;

    	Bottle &blobBottle = blobsBottle.addList();
    	blobBottle.addInt(centroid[0]);
    	blobBottle.addInt(centroid[1]);
    	blobBottle.addInt((int)(blobSize+0.5f));

    	if (blobsOutPort.getOutputCount()>0)
    	{
    		blobsOutPort.prepare() = blobsBottle;

    		blobsOutPort.setEnvelope(stamp);
    		blobsOutPort.write();
    	}

    	/* Prepare and write output opt port */

    	if (optOutPort.getOutputCount()>0)
    	{

    		yarp::sig::ImageOf<yarp::sig::PixelMono> &blobImage = optOutPort.prepare();
    	    blobImage.resize(blobMat.cols, blobMat.rows);

    	    blobMat.copyTo( cv::Mat( (IplImage*)blobImage.getIplImage() ) );

    		optOutPort.setEnvelope(stamp);
    		optOutPort.write();
    	}

	/* Prepare and write output crop port */

    	if (cropOutPort.getOutputCount()>0)
    	{

    		int x = centroid[0];
    		int y = centroid[1];

    		int dx = ( (cropSize>0) ? cropSize : (roi[2]-roi[0]) );
    		int dy = ( (cropSize>0) ? cropSize : (roi[3]-roi[1]) );

    		int dx2 = dx>>1;
    		int dy2 = dy>>1;

    		cv::Point tl = cv::Point( std::max(x-dx2,0), std::max(y-dy2,0) );
    		cv::Point br = cv::Point( std::min(x+dx2,blobMat.rows-1), std::min(y+dy2,blobMat.cols-1) );

    		cv::Rect roiRegion = cv::Rect(tl,br);

    		yarp::sig::ImageOf<yarp::sig::PixelBgr> &cropImage = cropOutPort.prepare();
    		cropImage.resize(roiRegion.width, roiRegion.height);

    		imagesMatBuffer.back()(roiRegion).copyTo( cv::Mat( (IplImage*)cropImage.getIplImage() ) );

    		cropOutPort.setEnvelope(stamp);
    		cropOutPort.write();

    	}

    }

    imagesMatBuffer.erase(imagesMatBuffer.begin());

    mutex.post();

}
//empty line to make gcc happy
