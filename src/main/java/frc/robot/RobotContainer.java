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

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.xerosw.hid.XeroGamepad;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ReefLevel;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.auto.AutoModeBaseCmd;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.AbortCmd;
import frc.robot.commands.robot.EjectCmd;
import frc.robot.commands.robot.algaenet.AlgaeNetWhileMovingCmd;
import frc.robot.commands.robot.climb.ExecuteClimbCmd;
import frc.robot.commands.robot.climb.PrepClimbCmd;
import frc.robot.commands.robot.climb.StowClimberCmd;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.ExecuteRobotActionCmd;
import frc.robot.subsystems.brain.QueueRobotActionCmd;
import frc.robot.subsystems.brain.RobotAction;
import frc.robot.subsystems.brain.SetCoralSideCmd;
import frc.robot.subsystems.brain.SetLevelCmd;
import frc.robot.subsystems.climber.ClimberIO;
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
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.CalibrateCmd;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.OIIOHID;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVision.PoseEstimateConsumer;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight4;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.ReefUtil;
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
    private ClimberSubsystem climber_;
    private FunnelSubsystem funnel_;
    private BrainSubsystem brain_;

    // Choosers
    private final LoggedDashboardChooser<Command> autoChooser_;
    private final LoggedDashboardChooser<Command> tuningChooser_;

    // Controller
    private final XeroGamepad gamepad_ = new XeroGamepad(0);

    private Trigger testModeTrigger ;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {  
        testModeTrigger = new Trigger(() -> {
            return RobotState.isTest() && RobotState.isEnabled() ;
        }) ;
        
        ReefUtil.initialize();


        /**
         * Subsystem setup
         */
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
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
                            new CameraIOLimelight4(VisionConstants.backLimelightName, drivebase_::getRotation)
                    ) ;

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        funnel_ = new FunnelSubsystem(new FunnelIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        climber_ = new ClimberSubsystem(new ClimberIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    break;

                case PRACTICE:
                    drivebase_ = new Drive(
                            new GyroIOPigeon2(PracticeTunerConstants.DrivetrainConstants.Pigeon2Id,
                                    PracticeTunerConstants.kCANBus),
                            ModuleIOTalonFX::new,
                            PracticeTunerConstants.FrontLeft,
                            PracticeTunerConstants.FrontRight,
                            PracticeTunerConstants.BackLeft,
                            PracticeTunerConstants.BackRight,
                            PracticeTunerConstants.kSpeedAt12Volts);

                    vision_ = new AprilTagVision(
                            drivebase_::addVisionMeasurement,
                            new CameraIOLimelight4(VisionConstants.frontLimelightName, drivebase_::getRotation)
                    // new CameraIOLimelight(VisionConstants.backLimelightName,
                    // drivebase_::getRotation),
                    // new CameraIOLimelight(VisionConstants.leftLimelightName,
                    // drivebase_::getRotation)
                    );

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        funnel_ = new FunnelSubsystem(new FunnelIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        climber_ = new ClimberSubsystem(new ClimberIOHardware());
                    }
                    catch(Exception ex) {
                        subsystemCreateException(ex) ;
                    }

                    break;

                case SIMBOT:
                case XEROSIM:
                    // Sim robot, instantiate physics sim IO implementations
                    drivebase_ = new Drive(
                            new GyroIO() {
                            },
                            ModuleIOSim::new,
                            CompTunerConstants.FrontLeft,
                            CompTunerConstants.FrontRight,
                            CompTunerConstants.BackLeft,
                            CompTunerConstants.BackRight,
                            CompTunerConstants.kSpeedAt12Volts);

                    vision_ = new AprilTagVision(
                        PoseEstimateConsumer.ignore(),
                        new CameraIOPhotonSim("Front", VisionConstants.frontTransform,
                        drivebase_::getPose, true),
                        new CameraIOPhotonSim("Back", VisionConstants.backTransform,
                        drivebase_::getPose, true)
                    );

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        climber_ = new ClimberSubsystem(new ClimberIOHardware());
                    } catch (Exception ex) {
                        subsystemCreateException(ex);
                    }

                    try {
                        funnel_ = new FunnelSubsystem(new FunnelIOHardware());
                    } catch (Exception ex) {
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
                default -> 3;
            };

            CameraIO[] cams = new CameraIO[numCams];
            Arrays.fill(cams, new CameraIO() {
            });

            vision_ = new AprilTagVision(
                    drivebase_::addVisionMeasurement,
                    cams);
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

        if (climber_ == null) {
            climber_ = new ClimberSubsystem(new ClimberIO() {
            });
        }

        // OI Setup
        oi_ = new OISubsystem(new OIIOHID(2), gamepad_);

        brain_ = new BrainSubsystem(oi_, drivebase_, manipulator_, grabber_, climber_, funnel_);

        DriveCommands.configure(
            drivebase_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );        

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
        configureTestModeBindings() ;

        manipulator_.setDefaultCommand(new CalibrateCmd(manipulator_));
    }

    public Drive drivebase() {
        return drivebase_;
    }

    public XeroGamepad gamepad() {
        return gamepad_;
    }

    public void setupAutos() {

        autoChooser_.addDefaultOption("Do Nothing", new AutoModeBaseCmd("Do Nothing")) ;

        autoChooser_.addOption("Left Side Coral (2 Coral)",
            AutoCommands.twoCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true));

        autoChooser_.addOption("Right Side Coral (2 Coral)",
            AutoCommands.twoCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, false));

        autoChooser_.addOption("Left Side Coral (3 Coral)",
            AutoCommands.threeCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true));

        autoChooser_.addOption("Right Side Coral (3 Coral)",
            AutoCommands.threeCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, false));

        autoChooser_.addOption("Center Algae Processor (1 Coral, 1 Algae)", 
            AutoCommands.oneCoralOneAlgaeProcessorAuto(brain_, drivebase_, manipulator_, grabber_));

        autoChooser_.addOption("Center Algae Barge (1 Coral, 1 Algae)", 
            AutoCommands.oneCoralOneAlgaeBargeAuto(brain_, drivebase_, manipulator_, grabber_));     
    }

    private void subsystemCreateException(Exception ex) {
        MessageLogger logger = MessageLogger.getTheMessageLogger();
        logger.startMessage(MessageType.Error);
        logger.add("Error creating subsystem", ex.getMessage());
        logger.endMessage();
        logger.logStackTrace(ex.getStackTrace());

        if (Constants.propogateExceptionOnSubsystemCreateFail) {
            throw new RuntimeException("Error creating subsystem", ex);
        }
    }

    private void configureTestModeBindings() {
        Distance h = Centimeters.of(140) ;
        gamepad_.start().and(testModeTrigger).onTrue(
            new ConditionalCommand(
                new GoToCmd(manipulator_, h, ManipulatorConstants.Arm.Positions.kRaiseAngle),
                new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kRaiseAngle),
                () -> manipulator_.getElevatorPosition().lt(ManipulatorConstants.Elevator.Positions.kPlaceL2))) ;    
        
    }

    boolean isArmOkToRaise() {
        Angle pos = manipulator_.getArmPosition() ;
        return pos.gte(Degrees.of(18.0)) && pos.lte(Degrees.of(178.0)) ;
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
        oi_.algaeReefKeep().onTrue(new QueueRobotActionCmd(brain_, RobotAction.CollectAlgaeReefKeep));
        oi_.algaeReefEject().onTrue(new QueueRobotActionCmd(brain_, RobotAction.CollectAlgaeReefEject)) ;
        oi_.algaeScore().onTrue(new QueueRobotActionCmd(brain_, RobotAction.ScoreAlgae));

        oi_.l1().onTrue(new SetLevelCmd(brain_, ReefLevel.L1).ignoringDisable(true));
        oi_.l2().onTrue(new SetLevelCmd(brain_, ReefLevel.L2).ignoringDisable(true));
        oi_.l3().onTrue(new SetLevelCmd(brain_, ReefLevel.L3).ignoringDisable(true));
        oi_.l4().onTrue(new SetLevelCmd(brain_, ReefLevel.L4).ignoringDisable(true));

        oi_.coralLeftRight().onTrue(new SetCoralSideCmd(brain_, CoralSide.Right).ignoringDisable(true));
        oi_.coralLeftRight().onFalse(new SetCoralSideCmd(brain_, CoralSide.Left).ignoringDisable(true));

        oi_.execute().onTrue(new ExecuteRobotActionCmd(brain_));

        oi_.abort().onTrue(new AbortCmd(brain_));
        oi_.eject().onTrue(new EjectCmd(brain_, manipulator_, grabber_));

        oi_.climbLock().negate().and(oi_.climbDeploy()).onTrue(new PrepClimbCmd(drivebase_, climber_, funnel_, manipulator_));
        oi_.climbLock().onTrue(new StowClimberCmd(manipulator_, climber_, funnel_)) ;
        oi_.climbLock().negate().and(oi_.climbExecute()).onTrue(new ExecuteClimbCmd(oi_, climber_, drivebase_, MetersPerSecond.of(0.25), Seconds.of(0.6))) ;
        
        climber_.readyToClimbTrigger().onTrue(gamepad_.setLockCommand(true)) ;
        oi_.rotateArm().onTrue(new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, Degrees.of(90.0))) ;

        oi_.raiseArm().and(this::isArmOkToRaise).onTrue(
            new GoToCmd(manipulator_, Feet.of(2.0), manipulator_.getArmPosition())
        ) ;

        oi_.algaeNet().onTrue(new AlgaeNetWhileMovingCmd(brain_, drivebase_, manipulator_, grabber_)) ;
    }

    /**
     * Sets up drivebase control mappings for drivers.
     */
    private void configureDriveBindings() {

        // Default command, normal field-relative drive
        drivebase_.setDefaultCommand(DriveCommands.joystickDrive());

        // Slow Mode, during left bumper
        gamepad_.leftBumper().whileTrue(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier));

        // Switch to X pattern / brake while X button is pressed
        gamepad_.x().whileTrue(drivebase_.stopWithXCmd());

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
                drivebase_.resetGyroCmd());

        gamepad_.rightTrigger().onTrue(new ExecuteRobotActionCmd(brain_));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command ret = null;

        if (Robot.useXeroSimulator()) {
            //
            // In the Xero simulator, set the auto mode you want to run
            // Note: the auto used here must match the simulation stimulus file set in the
            // Robot.java file.
            //

            // ret = AutoCommands.oneCoralAuto(brain_, drivebase_, manipulator_, grabber_) ;
            ret = AutoCommands.threeCoralSideAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true) ;
            // ret = AutoCommands.oneCoralOneAlgaeAuto(brain_, drivebase_, manipulator_, grabber_) ;
            // ret = AutoCommands.twoCoralCenterAuto(brain_, drivebase_, manipulator_, grabber_, funnel_, true);

            // Command autoChosen = autoChooser_.get();
            // ret = autoChosen != null ? autoChosen : tuningChooser_.get();

        } else {
            Command autoChosen = autoChooser_.get();
            ret = autoChosen != null ? autoChosen : tuningChooser_.get();
        }

        return ret;
    }
}
