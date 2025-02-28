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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Function;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.CompTunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.ReefUtil;
import frc.robot.util.ReefUtil.ReefFace;

public class Drive extends SubsystemBase {
    // These Constants should be the same for every drivebase, so just use the comp bot constants.
    static final double ODOMETRY_FREQUENCY = new CANBus(CompTunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public final double DRIVE_BASE_RADIUS;

    // These constants should change for every drivebase
    private final LinearVelocity SPEED_12_VOLTS;
    private final RobotConfig PP_CONFIG;
    
    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 63.5;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
    new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    
    private final SwerveDriveKinematics kinematics;
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator poseEstimator;

    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> flConfig_;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frConfig_;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> blConfig_;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> brConfig_;

    public Drive(
        GyroIO gyroIO,
        Function<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>, ModuleIO> moduleConstructor,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> flConfig,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frConfig,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> blConfig,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> brConfig,
        LinearVelocity kSpeed12Volts
    ) {
        // Setup Module IOs
        this.gyroIO = gyroIO;
        modules[0] = new Module(moduleConstructor.apply(flConfig), 0, flConfig);
        modules[1] = new Module(moduleConstructor.apply(frConfig), 1, frConfig);
        modules[2] = new Module(moduleConstructor.apply(blConfig), 2, blConfig);
        modules[3] = new Module(moduleConstructor.apply(brConfig), 3, brConfig);

        flConfig_ = flConfig;
        frConfig_ = frConfig;
        blConfig_ = blConfig;
        brConfig_ = brConfig;

        DRIVE_BASE_RADIUS = 
            Math.max(
            Math.max(
            Math.hypot(flConfig.LocationX, flConfig.LocationY),
            Math.hypot(frConfig.LocationX, frConfig.LocationY)),
            Math.max(
            Math.hypot(blConfig.LocationX, blConfig.LocationY),
            Math.hypot(brConfig.LocationX, brConfig.LocationY)));

        // Pose Estimators and Kinematics
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

        // PP Configurations
        SPEED_12_VOLTS = kSpeed12Volts;

        PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                flConfig.WheelRadius,
                SPEED_12_VOLTS.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1).withReduction(flConfig.DriveMotorGearRatio),
                flConfig.SlipCurrent,
                1
            ),
            getModuleTranslations()
        );
        
        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
        
        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();
        
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
            new PIDConstants(9.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            }
        );

        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            }
        );
        
        // Configure SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this)
        );
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();
        
        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
        
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }
        
        // Update odometry
        double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }
            
            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }
            
            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
        
        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);

        // If in a simulation or replay, log reef face selection information.
        if (Constants.getMode() != Mode.REAL) {
            Optional<ReefFace> face = ReefUtil.getTargetedReefFace(getPose());

            if (face.isPresent()) {
                Logger.recordOutput("ReefMath/NearestFace", new Pose2d[] {face.get().getAlgaeCollectPose()});
            } else {
                Logger.recordOutput("ReefMath/NearestFace", new Pose2d[] {});
            }
        }

        ChassisSpeeds spd = getChassisSpeeds() ;
        Logger.recordOutput("drive/velocity", Math.hypot(spd.vxMetersPerSecond, spd.vyMetersPerSecond)) ;
    }
    
    /**
    * Runs the drive at the desired velocity.
    *
    * @param speeds Speeds in meters/sec
    */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SPEED_12_VOLTS);
        
        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
        
        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
        
        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }
    
    /**
    * Runs the drive at the desired velocities.
    * 
    * @param x X Velocity
    * @param y Y Velocity
    * @param omega Angular Velocity
    */
    public void runVelocity(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
        runVelocity(new ChassisSpeeds(x, y, omega));
    }
    
    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }
    
    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }
    
    /**
    * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
    * return to their normal orientations the next time a nonzero velocity is requested.
    */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }
    
    // public void pointWheelsAt() {
    
    
    //   Rotation2d[] headings = new Rotation2d[4];
    //   for (int i = 0; i < 4; i++) {
    //     headings[i] = 
    //   }
    // }
    
    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
    }
    
    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Resets the gyroscope to be forward based on which alliance is present.
     * This allows drivers to always Y & B to the same direction no matter the alliance.
     * @return The command that does the stated
     */
    public Command smartResetGyro() {
        return defer(() -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return alliance == Alliance.Blue ? resetGyroCmd() : resetGyroCmd(Rotation2d.fromRotations(0.5));
        });
    }
    
    public Command resetGyroCmd(Rotation2d rotation) {
        return runOnce(() -> {
            setPose(new Pose2d(getPose().getTranslation(), rotation));
        }).ignoringDisable(true);
    }

    public Command resetGyroCmd() {
        return resetGyroCmd(new Rotation2d());
    }
    
    public Command stopWithXCmd() {
        return runOnce(() -> stopWithX());
    }
    
    public Command stopCmd() {
        return runOnce(() -> stop());
    }
    
    public Command runVelocityCmd(ChassisSpeeds speeds) {
        return run(() -> {
            runVelocity(speeds);
        });
    }
    
    public Command runVelocityCmd(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
        return run(() -> {
            runVelocity(x, y, omega);
        });
    }
    
    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }
    
    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }
    
    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
    
    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }
    
    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }
    
    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }
    
    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }
    
    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
    Pose2d visionRobotPoseMeters,
    double timestampSeconds,
    Matrix<N3, N1> visionMeasurementStdDevs) {
        Logger.recordOutput("Odometry/VisionMeasurement", visionRobotPoseMeters);
        poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
    
    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return SPEED_12_VOLTS.in(MetersPerSecond);
    }
    
    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    public RobotConfig getPathplannerConfig() {
        return PP_CONFIG;
    }
    
    /** Returns an array of module translations. */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(flConfig_.LocationX, flConfig_.LocationY),
            new Translation2d(frConfig_.LocationX, frConfig_.LocationY),
            new Translation2d(blConfig_.LocationX, blConfig_.LocationY),
            new Translation2d(brConfig_.LocationX, brConfig_.LocationY)
        };
    }
}