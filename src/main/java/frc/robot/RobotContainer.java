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

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.xerosw.hid.XeroGamepad;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.AbortCmd;
import frc.robot.commands.robot.EjectCmd;
import frc.robot.commands.robot.climb.ExecuteClimbCmd;
import frc.robot.commands.robot.climb.PrepClimbCmd;
import frc.robot.commands.robot.climb.StowClimberCmd;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.ExecuteRobotActionCmd;
import frc.robot.subsystems.brain.QueueRobotActionCmd;
import frc.robot.subsystems.brain.RobotAction;
import frc.robot.subsystems.brain.SetCoralSideCmd;
import frc.robot.subsystems.brain.SetLevelCmd;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOHardware;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberIO;
import frc.robot.subsystems.grabber.GrabberIOHardware;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.OIIOHID;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVision.PoseEstimateConsumer;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.CameraIOLimelight4;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.simulator.engine.ISimulatedSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer instance_;

    public static RobotContainer getInstance() {
        if (instance_ == null) {
            instance_ = new RobotContainer();
        }

        return instance_;
    }

    // Mapping of subsystems name to subsystems, used by the simulator
    HashMap<String, ISimulatedSubsystem> subsystems_ = new HashMap<>();

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
    private OISubsystem oi_;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    @SuppressWarnings("unused")
    private ClimberSubsystem climber_;
    private FunnelSubsystem funnel_;
    private BrainSubsystem brain_;

    // Choosers
    private final LoggedDashboardChooser<Command> autoChooser_;
    private final LoggedDashboardChooser<Command> tuningChooser_;

    // Controller
    private final XeroGamepad gamepad_ = new XeroGamepad(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        /**
         * Subsystem setup
         */
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ALPHA:
                    throw new RuntimeException("the alpha bot is no longer supported") ;
                    
                case COMPETITION:
                    drivebase_ = new Drive(
                            new GyroIOPigeon2(CompTunerConstants.DrivetrainConstants.Pigeon2Id, CompTunerConstants.kCANBus),
                            ModuleIOTalonFX::new,
                            CompTunerConstants.FrontLeft,
                            CompTunerConstants.FrontRight,
                            CompTunerConstants.BackLeft,
                            CompTunerConstants.BackRight,
                            CompTunerConstants.kSpeedAt12Volts);

                    vision_ = new AprilTagVision(
                            drivebase_::addVisionMeasurement,
                            new CameraIOLimelight4(VisionConstants.frontLimelightName, drivebase_::getRotation),
                            new CameraIOLimelight(VisionConstants.backLimelightName, drivebase_::getRotation),
                            new CameraIOLimelight(VisionConstants.leftLimelightName, drivebase_::getRotation));

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        funnel_ = new FunnelSubsystem(new FunnelIOHardware());
                    } 
                    catch (Exception ex) {
                        subsystemCreateException(ex);
                    }                    

                //     try {
                //         climber_ = new ClimberSubsystem(new ClimberIOHardware());
                //     }
                //     catch(Exception ex) {
                //         subsystemCreateException(ex) ;
                //     }


                    break;

                case PRACTICE:
                    drivebase_ = new Drive(
                            new GyroIOPigeon2(PracticeTunerConstants.DrivetrainConstants.Pigeon2Id, PracticeTunerConstants.kCANBus),
                            ModuleIOTalonFX::new,
                            PracticeTunerConstants.FrontLeft,
                            PracticeTunerConstants.FrontRight,
                            PracticeTunerConstants.BackLeft,
                            PracticeTunerConstants.BackRight,
                            PracticeTunerConstants.kSpeedAt12Volts);

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOLimelight4(VisionConstants.frontLimelightName, drivebase_::getRotation),
                        new CameraIOLimelight(VisionConstants.backLimelightName, drivebase_::getRotation),
                        new CameraIOLimelight(VisionConstants.leftLimelightName, drivebase_::getRotation)
                    );

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        funnel_ = new FunnelSubsystem(new FunnelIOHardware());
                    } 
                    catch (Exception ex) {
                        subsystemCreateException(ex);
                    }                    

                //     try {
                //         climber_ = new ClimberSubsystem(new ClimberIOHardware());
                //     }
                //     catch(Exception ex) {
                //         subsystemCreateException(ex) ;
                //     }

                    break;

                case SIMBOT:
                case XEROSIM:
                    // Sim robot, instantiate physics sim IO implementations
                    drivebase_ = new Drive(
                        new GyroIO() {},
                        ModuleIOSim::new,
                        CompTunerConstants.FrontLeft,
                        CompTunerConstants.FrontRight,
                        CompTunerConstants.BackLeft,
                        CompTunerConstants.BackRight,
                        CompTunerConstants.kSpeedAt12Volts
                    );

                    vision_ = new AprilTagVision(
                        PoseEstimateConsumer.ignore(),
                        new CameraIOPhotonSim("Front", VisionConstants.frontTransform, drivebase_::getPose, true),
                        new CameraIOPhotonSim("Back", VisionConstants.backTransform, drivebase_::getPose, false),
                        new CameraIOPhotonSim("Left", VisionConstants.leftTransform, drivebase_::getPose, false)
                    );

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        climber_ = new ClimberSubsystem(new ClimberIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    try {
                        funnel_ = new FunnelSubsystem(new FunnelIOHardware());
                    } 
                    catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    break;
            }
        }

        /**
         * Empty subsystem setup (required in replay)
         */
        if (drivebase_ == null) { // This will be null in replay, or whenever a case above leaves a subsystem
                                  // uninstantiated.
            switch (Constants.getRobot()) {
                case ALPHA:
                    drivebase_ = new Drive(
                            new GyroIO() {
                            },
                            ModuleIOReplay::new,
                            AlphaTunerConstants.FrontLeft,
                            AlphaTunerConstants.FrontRight,
                            AlphaTunerConstants.BackLeft,
                            AlphaTunerConstants.BackRight,
                            AlphaTunerConstants.kSpeedAt12Volts);
                    break;
                case PRACTICE:
                    drivebase_ = new Drive(
                            new GyroIO() {
                            },
                            ModuleIOReplay::new,
                            PracticeTunerConstants.FrontLeft,
                            PracticeTunerConstants.FrontRight,
                            PracticeTunerConstants.BackLeft,
                            PracticeTunerConstants.BackRight,
                            PracticeTunerConstants.kSpeedAt12Volts);
                    break;
                default: // SimBot or Comp Bot
                    drivebase_ = new Drive(
                            new GyroIO() {
                            },
                            ModuleIOReplay::new,
                            CompTunerConstants.FrontLeft,
                            CompTunerConstants.FrontRight,
                            CompTunerConstants.BackLeft,
                            CompTunerConstants.BackRight,
                            CompTunerConstants.kSpeedAt12Volts);
                    break;
            }
        }

        if (vision_ == null) {
            int numCams = switch (Constants.getRobot()) {
                case ALPHA -> 0;
                case PRACTICE -> 1;
                case COMPETITION -> 3;
                case SIMBOT -> 4;
                case XEROSIM -> 4;
            };

            CameraIO[] cams = new CameraIO[numCams];
            Arrays.fill(cams, new CameraIO() {});

            vision_ = new AprilTagVision(
                drivebase_::addVisionMeasurement,
                cams
            );
        }

        if (manipulator_ == null) {
            manipulator_ = new ManipulatorSubsystem(new ManipulatorIO() {
            });
        }

        if (grabber_ == null) {
            grabber_ = new GrabberSubsystem(new GrabberIO() {
            });
        }

        if (funnel_ == null) {
            funnel_ = new FunnelSubsystem(new FunnelIO() {
            });
        }

        // OI Setup
        oi_ = new OISubsystem(new OIIOHID(2), gamepad_);

        brain_ = new BrainSubsystem(oi_, drivebase_, manipulator_, grabber_, climber_);

        // Shuffleboard Tabs
        ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
        ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

        // Widgets & Choosers
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices");
        tuningChooser_ = new LoggedDashboardChooser<>("Tuning Choices");

        // Add choosers/widgets to tabs.
        autonomousTab.add("Auto Mode", autoChooser_.getSendableChooser()).withSize(2, 1);
        tuningTab.add("Tuning Modes", tuningChooser_.getSendableChooser()).withSize(2, 1);

        // Configure the button bindings
        configureDriveBindings();
        configureButtonBindings();
    }

    public Drive drivebase() {
        return drivebase_;
    }

    public XeroGamepad gamepad() {
        return gamepad_;
    }

    public void setupAutos() {

        autoChooser_.addDefaultOption("Do Nothing", Commands.none());
        autoChooser_.addOption("Alliance Side Coral",
                AutoCommands.threeCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true));
        autoChooser_.addOption("Opposing Side Coral",
                AutoCommands.threeCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, false));
        autoChooser_.addOption("Center Algae", AutoCommands.oneCoralOneAlgaeAuto(brain_, drivebase_, manipulator_, grabber_));
        autoChooser_.addOption("Center Coral (alliance side station)",
                AutoCommands.twoCoralCenterAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true));
        autoChooser_.addOption("Center Coral (opposing side station)",
                AutoCommands.twoCoralCenterAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, false));
        autoChooser_.addOption("Just Coral (center)", AutoCommands.oneCoralAuto(brain_, drivebase_, manipulator_, grabber_));
        autoChooser_.addOption("Fallback To Tuning Chooser (SW ONLY)", null);

        tuningChooser_.addOption("Straight Tuning Path", DriveCommands.initialFollowPathCommand(drivebase_, "Tuning Path Straight"));
        tuningChooser_.addOption("Curved Tuning Path", DriveCommands.initialFollowPathCommand(drivebase_, "Tuning Path Curved"));

        // Add SysId routines to the chooser
        tuningChooser_.addOption("Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drivebase_));
        tuningChooser_.addOption("Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drivebase_));
        tuningChooser_.addOption("Drive SysId (Quasistatic Forward)",
                drivebase_.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tuningChooser_.addOption("Drive SysId (Quasistatic Reverse)",
                drivebase_.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        tuningChooser_.addOption("Drive SysId (Dynamic Forward)",
                drivebase_.sysIdDynamic(SysIdRoutine.Direction.kForward));
        tuningChooser_.addOption("Drive SysId (Dynamic Reverse)",
                drivebase_.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    private void subsystemCreateException(Exception ex) {
        MessageLogger logger = MessageLogger.getTheMessageLogger() ;
        logger.startMessage(MessageType.Error) ;
        logger.add("Error creating subsystem", ex.getMessage());
        logger.endMessage() ;
        logger.logStackTrace(ex.getStackTrace()) ;

        if (Constants.propogateExceptionOnSubsystemCreateFail) {
            throw new RuntimeException("Error creating subsystem", ex);
        }
    }

    /**
     * Use this method to define your button -> command mappings for drivers.
     */
    private void configureButtonBindings() {
        //
        // These are the bindings for the various operations of the robot
        //
        oi_.coralPlace().onTrue(new QueueRobotActionCmd(brain_, RobotAction.PlaceCoral));
        oi_.coralCollect().onTrue(new QueueRobotActionCmd(brain_, RobotAction.CollectCoral));
        oi_.algaeReef().onTrue(new QueueRobotActionCmd(brain_, RobotAction.CollectAlgaeReef));
        oi_.algaeGround().onTrue(new QueueRobotActionCmd(brain_, RobotAction.CollectAlgaeGround));
        oi_.algaeScore().onTrue(new QueueRobotActionCmd(brain_, RobotAction.ScoreAlgae));

        oi_.l1().onTrue(new SetLevelCmd(brain_, ReefLevel.L1).ignoringDisable(true));
        oi_.l2().onTrue(new SetLevelCmd(brain_, ReefLevel.L2).ignoringDisable(true));
        oi_.l3().onTrue(new SetLevelCmd(brain_, ReefLevel.L3).ignoringDisable(true));
        oi_.l4().onTrue(new SetLevelCmd(brain_, ReefLevel.L4).ignoringDisable(true));

        //
        // Disable this for now until we have better data on whether this is an issue
        //
        // oi_.algaeOnReefTrigger().onTrue(Commands.runOnce(()-> brain_.toggleAlgaeOnReef()).ignoringDisable(true)) ;

        oi_.coralLeftRight().onTrue(new SetCoralSideCmd(brain_, CoralSide.Right).ignoringDisable(true));
        oi_.coralLeftRight().onFalse(new SetCoralSideCmd(brain_, CoralSide.Left).ignoringDisable(true));

        oi_.execute().onTrue(new ExecuteRobotActionCmd(brain_));

        oi_.abort().onTrue(new AbortCmd(brain_)) ;
        oi_.eject().onTrue(new EjectCmd(brain_, manipulator_, grabber_)) ;
        

        oi_.climbLock().onFalse(new PrepClimbCmd(climber_, funnel_)) ;
        oi_.climbLock().onTrue(new StowClimberCmd(climber_, funnel_)) ;
        oi_.climbExecute().onTrue(new ExecuteClimbCmd(climber_)) ;

        oi_.climbExecute().and(climber_.readyToClimbTrigger()).onTrue(vision_.setEnabledCommand(true)) ;
        oi_.climbDeploy().onTrue(vision_.setEnabledCommand(false)) ;
    }

    /**
     * Sets up drivebase control mappings for drivers.
     */
    private void configureDriveBindings() {

        // Default command, normal field-relative drive
        drivebase_.setDefaultCommand(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> -gamepad_.getLeftY(),
                () -> -gamepad_.getLeftX(),
                () -> -gamepad_.getRightX()
            )
        );
        
        // Slow Mode, during left bumper
        gamepad_.leftBumper().whileTrue(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier
            )
        );

        // Switch to X pattern / brake while X button is pressed
        gamepad_.x().whileTrue(drivebase_.stopWithXCmd());
        gamepad_.a().onTrue(new ExecuteRobotActionCmd(brain_)) ;

        // Robot Relative
        gamepad_.povUp().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

        gamepad_.povDown().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0),
                        RadiansPerSecond.zero()));

        gamepad_.povLeft().whileTrue(
                drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero()));

        gamepad_.povRight().whileTrue(
                drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(),
                        RadiansPerSecond.zero()));

        // Robot relative diagonal
        gamepad_.povUpLeft().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero()));

        gamepad_.povUpRight().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero()));

        gamepad_.povDownLeft().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero()));

        gamepad_.povDownRight().whileTrue(
                drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero()));

        // Reset gyro to 0° when Y & B button is pressed
        gamepad_.y().and(gamepad_.b()).onTrue(
            drivebase_.resetGyroCmd()
        );

        gamepad_.y().and(gamepad_.a()).and(gamepad_.rightBumper()).onTrue(
            drivebase_.resetGyroCmd(new Rotation2d(Rotations.of(0.5)))
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command ret = null ;

        if (Robot.useXeroSimulator()) {
            //
            // In the Xero simulator, set the auto mode you want to run
            // Note: the auto used here must match the simulation stimulus file set in the Robot.java file.
            //

            // ret = AutoCommands.oneCoralAuto(brain_, drivebase_, manipulator_, grabber_) ;
            // ret = AutoCommands.threeCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true) ;
            // ret = AutoCommands.oneCoralOneAlgaeAuto(brain_, drivebase_, manipulator_, grabber_) ;
            ret = AutoCommands.twoCoralCenterAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true) ;
        }
        else {
            Command autoChosen = autoChooser_.get();
            ret = autoChosen != null ? autoChosen : tuningChooser_.get();
        }

        return ret;
    }
}
