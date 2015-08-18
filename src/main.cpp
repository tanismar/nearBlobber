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

#include "dispBlobberModule.hpp"

using namespace yarp::os;

int main(int argc, char * argv[])
{

    Network::init();

    DispBlobberModule module;

    ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "dispBlobber/conf" );
    rf.setDefaultConfigFile( "dispBlobber.ini" );
    rf.setDefault("name","dispBlobber");
    rf.configure( argc, argv );

    module.runModule(rf);

    Network::fini();

    return 0;
}
//empty line to make gcc happy
