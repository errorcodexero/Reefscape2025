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
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.gps.AbortCmd;
import frc.robot.commands.gps.CollectCoralCmd;
import frc.robot.commands.gps.CollectReefAlgaeAfterCmd;
import frc.robot.commands.gps.CollectReefAlgaeBeforeCmd;
import frc.robot.commands.gps.EjectCmd;
import frc.robot.commands.gps.PlaceCoralAfterCmd;
import frc.robot.commands.gps.PlaceCoralBeforeCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.brain.Brain;
import frc.robot.subsystems.brain.ExecuteRobotAction;
import frc.robot.subsystems.brain.QueueRobotAction;
import frc.robot.subsystems.climber.ClimbExecuteCmd;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.DeployClimberCmd;
import frc.robot.subsystems.climber.RetractClimberCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.funnel.FunnelIOHardware;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberIOHardware;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.oi.OICommandSupplier;
import frc.robot.subsystems.oi.OIConstants;
import frc.robot.subsystems.oi.OIIOHID;
import frc.robot.subsystems.oi.OISubsystem;
import frc.robot.subsystems.oi.CoralSide;
import frc.robot.subsystems.oi.OISubsystem.LEDState;
import frc.robot.subsystems.oi.RobotAction;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.CameraIOPhotonSim;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    
    public enum GamePiece {
        NONE,
        CORAL,
        ALGAE_HIGH,
        ALGAE_LOW
    } ;

    private static RobotContainer container_ ;

    public static RobotContainer getRobotContainer() {
        if (container_ == null) {
            container_ = new RobotContainer() ;
        }

        return container_ ;
    }

    // Mapping of subsystems name to subsystems, used by the simulator
    private boolean driver_controller_enabled_ = true ;

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
    private OISubsystem oi_ ;
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;
    private Brain brain_ ;

    private GamePiece holding_ ;

    
    // Controller
    private final CommandXboxController gamepad_ = new CommandXboxController(OIConstants.kGamepadPort) ;
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser_;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (RobotContainer.container_ != null) {
            throw new RuntimeException("Robot code tried to create multiple robot containers") ;
        }

        RobotContainer.container_ = this ;

        /**
         * Subsystem setup
         */
        if (Constants.getMode() != Mode.REPLAY) {
            oi_ = new OISubsystem(new OIIOHID(OIConstants.kOIPort), gamepad_) ;
            brain_ = new Brain(oi_, this::getRobotActionCommand) ;

            switch (Constants.getRobot()) {
                case COMPETITION:
                    break;
                
                case PRACTICE:
                    try {
                        drivebase_ =
                            new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft, TunerConstants.DrivetrainConstants.CANBusName),
                                new ModuleIOTalonFX(TunerConstants.FrontRight, TunerConstants.DrivetrainConstants.CANBusName),
                                new ModuleIOTalonFX(TunerConstants.BackLeft, TunerConstants.DrivetrainConstants.CANBusName),
                                new ModuleIOTalonFX(TunerConstants.BackRight, TunerConstants.DrivetrainConstants.CANBusName));
                    }
                    catch(Exception e) {
                    }

                    try {
                        vision_ = new AprilTagVision(
                            drivebase_::addVisionMeasurement,
                            new CameraIOLimelight(VisionConstants.frontLimelightName),
                            new CameraIOLimelight(VisionConstants.backLimelightName));
                    }
                    catch(Exception e) {
                    }

                    try {
                        manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware()) ;
                    }
                    catch(Exception e) {
                    }

                    try {
                        grabber_ = new GrabberSubsystem(new GrabberIOHardware()) ;
                    }
                    catch(Exception e) {
                    }

                    // try {
                    //     climber_ = new ClimberSubsystem(new ClimberIOHardware()) ;
                    // }
                    // catch(Exception e) {
                    // }

                    // try {
                    //     funnel_ = new FunnelSubsystem(new FunnelIOHardware()) ;
                    // }
                    // catch(Exception e) {

                    // }
                            
                    break;
                
                case SIMBOT:
                    // Sim robot, instantiate physics sim IO implementations
                    drivebase_ =
                        new Drive(
                            new GyroIO() {},
                            new ModuleIOSim(TunerConstants.FrontLeft),
                            new ModuleIOSim(TunerConstants.FrontRight),
                            new ModuleIOSim(TunerConstants.BackLeft),
                            new ModuleIOSim(TunerConstants.BackRight));

                    vision_ = new AprilTagVision(
                        (Pose2d robotPose, double timestampSecnds, Matrix<N3, N1> standardDeviations) -> {},
                        new CameraIOPhotonSim("Front", new Transform3d(
                            new Translation3d(Inches.of(14), Inches.zero(), Centimeters.of(20)),
                            new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.zero())
                        ), drivebase_::getPose),
                        new CameraIOPhotonSim("Back", new Transform3d(
                            new Translation3d(Inches.of(-14), Inches.zero(), Centimeters.of(20)),
                            new Rotation3d(Degrees.zero(), Degrees.of(-30), Rotations.of(0.5))
                        ), drivebase_::getPose));

                    manipulator_ = new ManipulatorSubsystem(new ManipulatorIOHardware()) ;
                    grabber_ = new GrabberSubsystem(new GrabberIOHardware()) ;
                    climber_ = new ClimberSubsystem(new ClimberIOHardware()) ;
                    funnel_ = new FunnelSubsystem(new FunnelIOHardware()) ;

                    break;
            }
        }

        /**
         * Empty subsystem setup (required in replay)
         */
        if (drivebase_ == null) { // This will be null in replay, or whenever a case above leaves a subsystem uninstantiated.
            drivebase_ =
                new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});

        }
        
        if (vision_ == null) {
            vision_ = new AprilTagVision(
                drivebase_::addVisionMeasurement,
                new CameraIO() {},
                new CameraIO() {});
        }

        // Set up auto chooser
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Add mirrored autos
        
        // Add SysId routines to the chooser
        autoChooser_.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drivebase_));
        autoChooser_.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drivebase_));
        autoChooser_.addOption("Drive SysId (Quasistatic Forward)", drivebase_.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser_.addOption("Drive SysId (Quasistatic Reverse)", drivebase_.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser_.addOption("Drive SysId (Dynamic Forward)", drivebase_.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser_.addOption("Drive SysId (Dynamic Reverse)", drivebase_.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // Configure the button bindings
        if (Constants.isCharacterization()) {
            configureCharBindings() ;
        }
        else {
            configureDriveBindings();

            // gamepad_.leftTrigger().onTrue(new WaitForCoralCmd(grabber_)) ;
            // gamepad_.leftTrigger().onTrue(new ManualPlaceReadyCmd(manipulator_, 3, true)) ;
            // gamepad_.leftTrigger().onTrue(new ManipulatorGrabAlgaeReefCmd(manipulator_, grabber_)) ;
            // gamepad_.rightTrigger().onTrue(new SetGrabberVelocityCmd(grabber_, Volts.of(6.0))) ;
            // gamepad_.a().onTrue(new ManipulatorGotoCmd(manipulator_, ManipulatorConstants.Elevator.kMinHeight.plus(Centimeters.of(0)), Degrees.of(0.0)));
            
            configureButtonBindings();
        }
    }

    public Drive drivebase() {
        return drivebase_ ;
    }

    public ManipulatorSubsystem manipulator() {
        return manipulator_ ;
    }

    public GrabberSubsystem grabber() {
        return grabber_ ;
    }

    public ClimberSubsystem climber() {
        return climber_ ;
    }

    public GamePiece holding() {
        return holding_ ;
    }

    public Brain getExecutor() {
        return brain_ ;
    }

    public void holding(GamePiece gp) {

        holding_ = gp ;
        switch(gp) {
            case NONE:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.Off) ;
                break ;

            case CORAL:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.On) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.Off) ;
                break ;
                
            case ALGAE_HIGH:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.On) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.Off) ;
                break ;

            case ALGAE_LOW:
                oi_.setLEDState(OISubsystem.OILed.HoldingCoral, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeHigh, LEDState.Off) ;
                oi_.setLEDState(OISubsystem.OILed.HoldingAlgaeLow, LEDState.On) ;
                break ;
        }
    }

    public OICommandSupplier.Pair<Command, Command> getRobotActionCommand(RobotAction action, int level, CoralSide side) {
        OICommandSupplier.Pair<Command, Command> ret = null ;

        switch(action) {
            case CollectCoral:
                ret = new OICommandSupplier.Pair<>(new CollectCoralCmd(manipulator_, grabber_), null) ;
                break ;

            case PlaceCoral:
                ret = new OICommandSupplier.Pair<>(
                            new PlaceCoralBeforeCmd(manipulator_, level),
                            new PlaceCoralAfterCmd(brain_, drivebase_, manipulator_, grabber_, false)) ;
                break ;

            case PlaceAlgae:
                // TODO: write me
                break ;

            case CollectAlgaeReefL2:
            ret = new OICommandSupplier.Pair<>(
                new CollectReefAlgaeBeforeCmd(manipulator_, level),
                new CollectReefAlgaeAfterCmd(brain_, drivebase_, manipulator_, grabber_, false)) ;
                break ;

            case CollectAlgaeReefL3:
                // TODO: write me
                break ;

            case CollectAlgaeGround:
                // TODO: write me
                break ;
        }

        return ret ;
    }

    public void enableGamepad(boolean enabled) {
        this.driver_controller_enabled_ = enabled ;
    }

    private void configureCharBindings() {
        // gamepad_.a().whileTrue(manipulator_.armSysIdQuasistatic(Direction.kForward)) ;
        // gamepad_.b().whileTrue(manipulator_.armSysIdQuasistatic(Direction.kReverse)) ;
        // gamepad_.x().whileTrue(manipulator_.armSysIdDynamic(Direction.kForward)) ;
        // gamepad_.y().whileTrue(manipulator_.armSysIdDynamic(Direction.kReverse)) ;

        // gamepad_.a().whileTrue(manipulator_.elevatorSysIdQuasistatic(Direction.kForward)) ;
        // gamepad_.b().whileTrue(manipulator_.elevatorSysIdQuasistatic(Direction.kReverse)) ;
        // gamepad_.x().whileTrue(manipulator_.elevatorSysIdDynamic(Direction.kForward)) ;
        // gamepad_.y().whileTrue(manipulator_.elevatorSysIdDynamic(Direction.kReverse)) ;

        gamepad_.a().whileTrue(grabber_.grabberSysIdQuasistatic(Direction.kForward)) ;
        gamepad_.b().whileTrue(grabber_.grabberSysIdQuasistatic(Direction.kReverse)) ;
        gamepad_.x().whileTrue(grabber_.grabberSysIdDynamic(Direction.kForward)) ;
        gamepad_.y().whileTrue(grabber_.grabberSysIdDynamic(Direction.kReverse)) ;
    }
    
    /**
    * Use this method to define your button -> command mappings for drivers.
    */
    private void configureButtonBindings() {
        oi_.climbLock().onTrue(new DeployClimberCmd(climber_, funnel_)) ;
        oi_.climbLock().onFalse(new RetractClimberCmd(climber_, funnel_)) ;

        oi_.climbExecute().onTrue(new ClimbExecuteCmd(climber_)) ;
        oi_.abort().onTrue(new AbortCmd()) ;
        oi_.eject().onTrue(new EjectCmd(manipulator_, grabber_)) ;

        oi_.coralPlace().onTrue(new QueueRobotAction(brain_, RobotAction.PlaceCoral)) ;
        oi_.coralCollect().onTrue(new QueueRobotAction(brain_, RobotAction.CollectCoral)) ;
        oi_.algaeCollectL2().onTrue(new QueueRobotAction(brain_, RobotAction.CollectAlgaeReefL2)) ;
        oi_.algaeCollectL3().onTrue(new QueueRobotAction(brain_, RobotAction.CollectAlgaeReefL3)) ;
        oi_.algaeGround().onTrue(new QueueRobotAction(brain_, RobotAction.CollectAlgaeGround)) ;
        oi_.algaeScore().onTrue(new QueueRobotAction(brain_, RobotAction.PlaceAlgae)) ;
        oi_.execute().onTrue(new ExecuteRobotAction(brain_)) ;
    }

    private double getLeftX() {
        if (!driver_controller_enabled_)
            return 0.0 ;

        double y = -gamepad_.getLeftX() ;
        y = Math.signum(y) * y * y ;
        
        return y ;
    }

    private double getLeftY() {
        if (!driver_controller_enabled_)
            return 0.0 ;

        double x = -gamepad_.getLeftY() ;
        x = Math.signum(x) * x * x;

        return x ;
    }

    private double getRightX() {
        if (!driver_controller_enabled_)
            return 0.0 ;

        double x = -gamepad_.getRightX() ;
        x = Math.signum(x) * x * x  ;

        return x ;
    }

    
    /**
     * Sets up drivebase control mappings for drivers.
     */
    private void configureDriveBindings() {
        // Default command, normal field-relative drive
        drivebase_.setDefaultCommand(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> getLeftY(),
                () -> getLeftX(),
                () -> getRightX())) ;
        
        // Slow Mode, during left bumper
        gamepad_.leftBumper().whileTrue(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> getRightX() * DriveConstants.slowModeJoystickMultiplier));
        
        // Switch to X pattern / brake while X button is pressed
        gamepad_.x().whileTrue(drivebase_.stopWithXCmd());
        
        // Robot Relative
        gamepad_.povUp().whileTrue(
            drivebase_.runVelocityCmd(FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero())
        );
        
        gamepad_.povDown().whileTrue(
            drivebase_.runVelocityCmd(FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0), RadiansPerSecond.zero())
        );
        
        gamepad_.povLeft().whileTrue(
            drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero())
        );
        
        gamepad_.povRight().whileTrue(
            drivebase_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(), RadiansPerSecond.zero())
        );

        // Robot relative diagonal
        gamepad_.povUpLeft().whileTrue(
            drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero())
        );

        gamepad_.povUpRight().whileTrue(
            drivebase_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero())
        );
        
        gamepad_.povDownLeft().whileTrue(
            drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero())
        );

        gamepad_.povDownRight().whileTrue(
            drivebase_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero())
        );
        
        // Reset gyro to 0° when Y & B button is pressed
        gamepad_.y().and(gamepad_.b()).onTrue(drivebase_.resetGyroCmd());
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return autoChooser_.get();
    }
}
