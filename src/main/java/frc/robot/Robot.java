// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.xerosw.util.MessageDestination;
import org.xerosw.util.MessageDestinationThumbFile;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;
import org.xerosw.util.RobotTimeSource;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoModeBaseCmd;
import frc.robot.commands.misc.StateCmd;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.simulator.engine.SimulationEngine;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends LoggedRobot {
    private HashMap<String, String> output_state_ ;

    private Command autonomousCommand;
    private RobotContainer robotContainer;
    
    private boolean hasSetupAutos = false;
    private AutoModeBaseCmd auto_cmd_ = null ;

    public Robot() throws RuntimeException {
        //
        // This is a hack, but it is the least intrusive approach based on where we are in the season
        //
        output_state_ = new HashMap<String, String>() ;
        StateCmd.setRobot(this);

        enableMessageLogger();

        MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("Robot code starting").endMessage() ;

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("Robot", Constants.getRobot().toString());
        Logger.recordMetadata("Mode", Constants.getMode().toString());
        Logger.recordMetadata("SimulationType", Robot.useXeroSimulator() ? "Xero Sim" : "Regular Sim");
        
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        
        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            
            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());

                // Silence Joystick Warnings
                DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());
                break;
            
            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
                break;
        }
        
        // Start AdvantageKit logger
        Logger.start();

        // Check for valid swerve config
        var modules =
        new SwerveModuleConstants[] {
            CompTunerConstants.FrontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight
        };

        for (var constants : modules) {
            if (
                constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated ||
                constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated
            ) {
                throw new RuntimeException("You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
            }
        }

        if (Robot.useXeroSimulator()) {
            String str = "threeCoralSideAuto" ;
            SimulationEngine.initializeSimulator(this);
            SimulationEngine.getInstance().initAll(str);
        }        
        
        //
        // For a one second delay.  We try 10 times to get a one second delay and if 
        // our delay is interrupted, we start over.  This delay is here because we see a race conditon
        // at times that causes the Pidgeon 2 to not get initialized.
        //
        // boolean done = false ;
        // for(int i = 0 ; i < 10 && !done ; i++) {
        //     done = true ;
        //     try {
        //         Thread.sleep(1000) ;
        //     }
        //     catch(Exception ex) {
        //         done = false ;
        //     }
        // }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.        
        robotContainer = RobotContainer.getInstance() ;
    }

    public static boolean useXeroSimulator() {
        return Constants.getRobot() == Constants.RobotType.XEROSIM;
    }

    public void setNamedState(String key, String value) {
        output_state_.put(key, value) ;
    }

    public void robotInit() {
        super.robotInit() ;

        if (Robot.useXeroSimulator() && SimulationEngine.getInstance() != null) {
            //
            // If we are simulating, create the simulation modules required
            //
            SimulationEngine.getInstance().createModels();
        }
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);
        
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);

        for(String key : output_state_.keySet()) {
            Logger.recordOutput("RobotState/" + key, output_state_.get(key)) ;
        }
    }
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        if(!hasSetupAutos && DriverStation.getAlliance().isPresent()) {
            robotContainer.setupAutos();
            hasSetupAutos = true;
        }

        if (hasSetupAutos) {
            Command cmd = robotContainer.getAutonomousCommand();
            if (cmd != null) {
                AutoModeBaseCmd autoCmd = (AutoModeBaseCmd) cmd;
                if (autoCmd != null) {
                    Drive d = RobotContainer.getInstance().drivebase() ;
                    Pose2d autopose = autoCmd.getStartingPose() ;
                    if (auto_cmd_ == null || auto_cmd_ != autoCmd) {
                        Logger.recordOutput("automode/name", autoCmd.getName()) ;
                        Logger.recordOutput("automode/pose", autopose) ;
                        d.setPose(autopose) ;
                        auto_cmd_ = autoCmd ;
                    }
                }
            }
        }
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
            
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        RobotContainer.getInstance().gamepad().setLocked(false);
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
    
    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        if (Robot.useXeroSimulator()) {
            SimulationEngine engine = SimulationEngine.getInstance();
            if (engine != null) {
                engine.run(getPeriod());
            }
        }
    }
    
    private void enableMessageLogger() {
        MessageDestination dest ;

        MessageLogger logger = MessageLogger.getTheMessageLogger() ;
        logger.setTimeSource(new RobotTimeSource());

        String logpath = null ;

        if (Robot.isSimulation()) {
            logpath = "logs" ;
        } else {
            logpath = "/u" ;
        }   

        dest = new MessageDestinationThumbFile(logpath, 250, RobotBase.isSimulation());
        logger.addDestination(dest);
    }
}
