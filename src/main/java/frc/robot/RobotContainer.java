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

import java.util.HashMap;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
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
    private final Drive drive_;
    
    @SuppressWarnings("unused")
    private final AprilTagVision vision_;
    
    // Controller
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser_;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive_ =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                vision_ = new AprilTagVision(
                    drive_::addVisionMeasurement,
                    new CameraIOLimelight("limelightfront"),
                    new CameraIOLimelight("limelightback"));
                    
                break;
            
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive_ =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                // TODO: Replace these transforms with accurate ones once we know the design
                vision_ = new AprilTagVision(
                    (Pose2d robotPose, double timestampSecnds, Matrix<N3, N1> standardDeviations) -> {},
                    new CameraIOPhotonSim("Front", new Transform3d(
                        new Translation3d(Inches.of(14), Inches.zero(), Centimeters.of(20)),
                        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.zero())
                    ), drive_::getPose),
                    new CameraIOPhotonSim("Back", new Transform3d(
                        new Translation3d(Inches.of(-14), Inches.zero(), Centimeters.of(20)),
                        new Rotation3d(Degrees.zero(), Degrees.of(-30), Rotations.of(0.5))
                    ), drive_::getPose));
                    
                break;
            
            default:
                // Replayed robot, disable IO implementations
                drive_ =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});

                vision_ = new AprilTagVision(
                    drive_::addVisionMeasurement,
                    new CameraIO() {},
                    new CameraIO() {});
                
                break;
        }
        this.addSubsystem(drive_) ;

        // Set up auto chooser
        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        
        // Add SysId routines to the chooser
        autoChooser_.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive_));
        autoChooser_.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive_));
        autoChooser_.addOption("Drive SysId (Quasistatic Forward)", drive_.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser_.addOption("Drive SysId (Quasistatic Reverse)", drive_.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser_.addOption("Drive SysId (Dynamic Forward)", drive_.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser_.addOption("Drive SysId (Dynamic Reverse)", drive_.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
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
        drive_.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive_,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()));
        
        // Slow Mode, during left bumper
        gamepad_.leftBumper().whileTrue(
        DriveCommands.joystickDrive(
            drive_,
            () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
            () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
            () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier));
        
        // Switch to X pattern / brake while X button is pressed
        gamepad_.x().whileTrue(drive_.stopWithXCmd());
        
        // Robot Relative
        gamepad_.povUp().whileTrue(
            drive_.runVelocityCmd(FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero())
        );
        
        gamepad_.povDown().whileTrue(
            drive_.runVelocityCmd(FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0), RadiansPerSecond.zero())
        );
        
        gamepad_.povLeft().whileTrue(
            drive_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero())
        );
        
        gamepad_.povRight().whileTrue(
            drive_.runVelocityCmd(MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(), RadiansPerSecond.zero())
        );

        // Robot relative diagonal
        gamepad_.povUp().and(gamepad_.povLeft()).whileTrue(
            drive_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero())
        );

        gamepad_.povUp().and(gamepad_.povRight()).whileTrue(
            drive_.runVelocityCmd(FeetPerSecond.of(0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero())
        );
        
        gamepad_.povDown().and(gamepad_.povLeft()).whileTrue(
            drive_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(0.707), RadiansPerSecond.zero())
        );

        gamepad_.povDown().and(gamepad_.povRight()).whileTrue(
            drive_.runVelocityCmd(FeetPerSecond.of(-0.707), FeetPerSecond.of(-0.707), RadiansPerSecond.zero())
        );
        
        // Reset gyro to 0° when Y & B button is pressed
        gamepad_.y().and(gamepad_.b()).onTrue(drive_.resetGyroCmd());
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
