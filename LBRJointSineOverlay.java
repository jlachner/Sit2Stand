package com.kuka.connectivity.fri.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.MotionCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * Creates a FRI Session.
 */
public class LBRJointSineOverlay extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "172.31.1.141";
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(5);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // move to start pose
        _lbr.move(ptp(Math.toRadians(50.94), Math.toRadians(66.26), Math.toRadians(8.67), Math.toRadians(-77.08), Math.toRadians(124.54), Math.toRadians(62.76), Math.toRadians(89.93)).setJointVelocityRel(0.2));

		// start PositionHold with overlay		
        PositionControlMode posCtrl = new PositionControlMode();
        
        PositionHold posHold = new PositionHold(posCtrl, -1, TimeUnit.MILLISECONDS);
        
        _lbr.move(posHold.addMotionOverlay(jointOverlay) );
        
        try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final LBRJointSineOverlay app = new LBRJointSineOverlay();
        app.runApplication();
    }

}
