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
#include "exp_robots.h"


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
std::ofstream openTxtFile(const std::string& basePath) {
    std::string filename = basePath;
    if (filename.size() < 4 || filename.substr(filename.size() - 4) != ".txt") {
        filename += ".txt";
    }
    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    return file;
}


//******************************************************************************
LBRJointSineOverlayClient::LBRJointSineOverlayClient()
    : _index(0)
{

    // Time variables for control loop
    currentTime = 0;
    sampleTime = 0;

    // Choose sit-to-stand or stand-to-sit trajectory; EDIT here
    // sit2stand = true; 

    // Initialize joint position; EDIT here for different initial poses
   // table against wall, aligned with sit-to-stand groove:
        qInitial[0] = -101.06 * M_PI/180;
        qInitial[1] = 104.70 * M_PI/180;
        qInitial[2] = -0.01 * M_PI/180;
        qInitial[3] = -53.22 * M_PI/180;
        qInitial[4] = -113.81 * M_PI/180;
        qInitial[5] = 81.67 * M_PI/180;
        qInitial[6] = -18.19 * M_PI/180;

    // if (sit2stand) {
    // // aligned with sit-to-stand groove:
    //     qInitial[0] = -86.98 * M_PI/180;
    //     qInitial[1] = 105.51 * M_PI/180;
    //     qInitial[2] = 26.71 * M_PI/180;
    //     qInitial[3] = -60.28 * M_PI/180;
    //     qInitial[4] = 65.55 * M_PI/180;
    //     qInitial[5] = 69.99 * M_PI/180;
    //     qInitial[6] = 22.75 * M_PI/180;
    // }
    // else {
        // // aligned with stand-to-sit groove:
        // qInitial[0] = -86.91 * M_PI/180;
        // qInitial[1] = 106.55 * M_PI/180;
        // qInitial[2] = 26.71 * M_PI/180;
        // qInitial[3] = -58.99 * M_PI/180;
        // qInitial[4] = 65.13 * M_PI/180;
        // qInitial[5] = 70.34 * M_PI/180;
        // qInitial[6] = 22.97 * M_PI/180;
    // }

    // Initialize joint positions
    for( int i=0; i < myLBR->nq; i++ )
    {
        qCurr[i] = qInitial[i];
        qOld[i] = qInitial[i];
    }

    // Use Explicit-cpp to create your robot
    myLBR = new iiwa14(1, "Dwight");
    myLBR->init( );

    q  = Eigen::VectorXd::Zero( myLBR->nq );
    dq  = Eigen::VectorXd::Zero( myLBR->nq );

    pointPosition = Eigen::Vector3d( 0.0, 0.0, 0.0722 );              // FT-Sensor plate + tool offset 72.2 mm -> 0.0722 m
    H = Eigen::MatrixXd::Zero( 4, 4 );

    // ************************************************************
    // INCLUDE FT-SENSOR
    // ************************************************************
    // Weight: 0.2kg (plate) + 0.255kg (sensor) = 0.455kg

    f_ext_ee = Eigen::VectorXd::Zero( 3 );
    m_ext_ee = Eigen::VectorXd::Zero( 3 );

    AtiForceTorqueSensor ftSensor("172.31.1.1");

    printf( "Force Sensor Activated. \n\n" );

    // ************************************************************
    // Store data
    // ************************************************************

    File_dt     = openTxtFile("/home/iiwaplayground/Documents/GitHub/Sit2Stand/robot_posCtrl/prints/File_dt");
    File_q      = openTxtFile("/home/iiwaplayground/Documents/GitHub/Sit2Stand/robot_posCtrl/prints/File_q");
    File_dq     = openTxtFile("/home/iiwaplayground/Documents/GitHub/Sit2Stand/robot_posCtrl/prints/File_dq");
    File_FExt   = openTxtFile("/home/iiwaplayground/Documents/GitHub/Sit2Stand/robot_posCtrl/prints/File_FExt");


    // ************************************************************
    // Initial print
    // ************************************************************

    printf( "Exp[licit](c)-cpp-FRI, https://explicit-robotics.github.io \n\n" );
    printf( "Robot '" );
    printf( "%s", myLBR->Name );
    printf( "' initialised. Ready to rumble! \n\n" );

    cout << "Loading joint position CSV data..." << endl;

    // Update with your CSV file path (relative or absolute); EDIT _q_data to follow different trajectory
    // if (sit2stand)
        _q_data = readCSV("../data/q_linear_z_sit_to_stand.csv").transpose();  // Now: N x 7
    // else
        // _q_data = readCSV("../data/q_linear_z_stand_to_sit.csv").transpose();  // Now: N x 7
    _N_data = _q_data.rows();

    cout << "Loaded " << _N_data << " trajectory points. \n\n" << endl;

    printf( "Waiting to connect with Server Application ... \n\n" );
}

//******************************************************************************
LBRJointSineOverlayClient::~LBRJointSineOverlayClient()
{

    File_dt.close();
    File_q.close();
    File_dq.close();
    File_FExt.close();
    delete this->ftSensor;

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
        sampleTime = robotState().getSampleTime();
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

    memcpy( qOld, qCurr, 7*sizeof(double) );
    memcpy( qCurr, robotState().getMeasuredJointPosition(), 7*sizeof(double) );

    for (int i=0; i < myLBR->nq; i++)
    {
        q[i] = qCurr[i];
    }

    for (int i=0; i < 7; i++)
    {
        dq[i] = (qCurr[i] - qOld[i]) / sampleTime;
    }

    // Get Forward Kinematics and extract rotation matrix
    H = myLBR->getForwardKinematics(q, 7, pointPosition);
    Eigen::Matrix3d R = H.block<3,3>(0,0);

    // Overwrite all joints from CSV data
    double jointPos[LBRState::NUMBER_OF_JOINTS];
    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
    {
        jointPos[i] = _q_data(_index, i);
    }

    robotCommand().setJointPosition(jointPos);

    // Only advance until last index, then hold
    if (_index < _N_data)
        _index++;

    // Read Force-Torque Sensor
    f_sens_ee = ftSensor->Acquire();

    // Assign sensor values to Eigen vectors
    f_ext_ee(0) = f_sens_ee[0];
    f_ext_ee(1) = f_sens_ee[1];
    f_ext_ee(2) = f_sens_ee[2];

    m_ext_ee(0) = f_sens_ee[3];
    m_ext_ee(1) = f_sens_ee[4];
    m_ext_ee(2) = f_sens_ee[5];

    // Transform forces and moments to base frame
    Eigen::Vector3d f_ext_0 = R * f_ext_ee;
    Eigen::Vector3d m_ext_0 = R * m_ext_ee;

    // Combine forces and moments into single vector
    Eigen::VectorXd F_ext_0(6);
    F_ext_0 << f_ext_0, m_ext_0; 

    // Write data to files
    File_dt << currentTime << endl;
    File_q << q << endl;
    File_dq << dq << endl;
    File_FExt << F_ext_0 << endl;

    // Update time
    currentTime = currentTime + sampleTime;

}

//******************************************************************************
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
