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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.simulator.engine.ISimulatedSubsystem;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

    // Mapping of subsystems name to subsystems, used by the simulator
    HashMap<String, ISimulatedSubsystem> subsystems_ = new HashMap<>() ;

    // Subsystems
    private Drive drivebase_;
    private AprilTagVision vision_;
    
    // Controller
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser_;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /**
         * Subsystem setup
         */
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ALPHA:

                    drivebase_ =
                        new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFX(TunerConstants.FrontLeft, TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.FrontRight, TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.BackLeft, TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.BackRight, TunerConstants.DrivetrainConstants.CANBusName));

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOLimelight(VisionConstants.frontLimelightName),
                        new CameraIOLimelight(VisionConstants.backLimelightName));
                        
                    break;

                case COMPETITION:

                    /** TODO: Instantiate Competition Subsystems, for now its a no-op. */

                    break;
                
                case PRACTICE:

                    drivebase_ =
                        new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFX(TunerConstants.FrontLeft, TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.FrontRight, TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.BackLeft, TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.BackRight, TunerConstants.DrivetrainConstants.CANBusName));

                    vision_ = new AprilTagVision(
                        drivebase_::addVisionMeasurement,
                        new CameraIOLimelight(VisionConstants.frontLimelightName),
                        new CameraIOLimelight(VisionConstants.backLimelightName));
                            
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

        // Simulation setup
        this.addSubsystem(drivebase_) ;

        // Set up auto chooser
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Add mirrored autos
        autoChooser_.addOption("Mirrored Example Auto", new PathPlannerAuto("test", true));
        
        // Add SysId routines to the chooser
        autoChooser_.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drivebase_));
        autoChooser_.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drivebase_));
        autoChooser_.addOption("Drive SysId (Quasistatic Forward)", drivebase_.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser_.addOption("Drive SysId (Quasistatic Reverse)", drivebase_.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser_.addOption("Drive SysId (Dynamic Forward)", drivebase_.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser_.addOption("Drive SysId (Dynamic Reverse)", drivebase_.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // Configure the button bindings
        configureDriveBindings();
        configureButtonBindings();
    }

    public ISimulatedSubsystem get(String name) {
        return this.subsystems_.get(name) ;
    }

    private void addSubsystem(SubsystemBase sub) {
        if (sub instanceof ISimulatedSubsystem) {
            this.subsystems_.put(sub.getName(),  (ISimulatedSubsystem)sub) ;
        }
    }
    
    /**
    * Use this method to define your button -> command mappings for drivers.
    */
    private void configureButtonBindings() {
        // Add subsystem button bindings here
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
            () -> -gamepad_.getRightX()));
        
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
