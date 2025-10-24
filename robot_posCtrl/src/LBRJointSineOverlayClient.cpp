/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.Connectivity FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.Connectivity FastRobotInterface� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2020 
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.17}
*/
#include <cstring>
#include <cstdio>
#include "LBRJointSineOverlayClient.h"
#include "friLBRState.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstring>
#include <chrono>


using Clock = std::chrono::high_resolution_clock;
static Clock::time_point last_time = Clock::now();

// Visual studio needs extra define to use math constants
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace KUKA::FRI;


//******************************************************************************
// A Function to read a CSV file and save it as a 2D matrix
// Dimensions for row x col are 7 x N, where 7 (row) is the degrees of freedom of the KUKA, N (col) is the number of data points
Eigen::MatrixXd readCSV( const string& filename )
{
    ifstream file( filename );
    if( !file.is_open( ) )
    {
        cerr << "Error: Couldn't open the file: " << filename << endl;
        exit( 1 );
    }

    // Read the values as 2D vector array
    vector<vector<double>> values;

    string line;
    int lineNum = 0;
    int numCols = 0;

    while ( getline( file, line ) )
    {
        ++lineNum;
        stringstream ss( line );
        string cell;
        vector<double> row;
        while ( getline( ss, cell, ',' ) )
        {
            try
            {
                row.push_back( stod( cell ) );
            } catch ( const std::invalid_argument& e )
            {
                cerr << "Error: Invalid argument at line " << lineNum << ", column: " << row.size() + 1 << endl;
                exit( 1 );
            }
        }

        values.push_back( row );
        if ( numCols == 0 )
        {
            numCols = row.size( );
        }
        else if ( row.size() != numCols )
        {
            cerr << "Error: Inconsistent number of columns in the CSV file." << endl;
            exit( 1 );
        }
    }

    // Error if CSV File is empty
    if ( values.empty( ) )
    {
        cerr << "Error: CSV file is empty." << endl;
        exit(1);
    }

    // Create Eigen Matrix
    Eigen::MatrixXd mat( values.size(), numCols );
    for (int i = 0; i < values.size(); i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            mat(i, j) = values[i][j];
        }
    }
    return mat;
}


//******************************************************************************
LBRJointSineOverlayClient::LBRJointSineOverlayClient()
    : _index(0)
{

    // ************************************************************
    // Initial print
    // ************************************************************

    printf( "LBR iiwa initialised. Ready to rumble! \n\n" );


    cout << "Loading joint position CSV data..." << endl;

    // Update with your CSV file path (relative or absolute)
    _q_data = readCSV("../data/q_linear_z.csv").transpose();  // Now: N x 7
    _N_data = _q_data.rows();

    cout << "Loaded " << _N_data << " trajectory points. \n\n" << endl;

    printf( "Waiting to connect with Server Application ... \n\n" );
}

//******************************************************************************
LBRJointSineOverlayClient::~LBRJointSineOverlayClient()
{
}
      
//******************************************************************************
void LBRJointSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
        _index = 0;  // Reset to start of trajectory
         break;
      }
      default:
      {
         break;
      }
   }
}
   
//******************************************************************************
void LBRJointSineOverlayClient::command()
{
    
    // Clamp at the last row to hold final pose
    if (_index >= _N_data)
        _index = _N_data - 1;

    double jointPos[LBRState::NUMBER_OF_JOINTS];
    memcpy(jointPos, robotState().getIpoJointPosition(), sizeof(jointPos));

    // Overwrite all joints from CSV data
    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
    {
        jointPos[i] = _q_data(_index, i);
    }

    robotCommand().setJointPosition(jointPos);

    // Only advance until last index, then hold
    if (_index < _N_data)
        _index++;


}

//******************************************************************************
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
