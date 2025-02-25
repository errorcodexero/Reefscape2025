package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CameraIOLimelight4 extends CameraIOLimelight {
    int current_imu_mode_ = -1 ;
    int current_ll_throttle_ = -1 ;

    /**
     * An enum representing the mode of the Limelight4's IMU. This is an enum for log readiblity.
     */
    public static enum IMUMode {
        NONE(0),
        IGNORING(0),
        SEEDING(1),
        USING(2),
        ASSIST_MT1(3),
        ASSIST_EXTERNAL(4);

        private final int id;

        private IMUMode(int id) {
            this.id = id;
        }
    }

    // Enabled Trigger
    private static final Trigger enabled = RobotModeTriggers.disabled().negate();

    // The current mode of the IMU, assuming this is not set anywhere else and IMU is enabled.
    private IMUMode currentMode_ = VisionConstants.useIMU ? IMUMode.SEEDING : IMUMode.IGNORING;

    // The current throttle of the LL4, assuming this is not set anywhere else and throttling is enabled.
    private int currentThrottle_ = VisionConstants.regulateThrottle ? VisionConstants.numSkippedFramesDisabled : 0;
    
    public CameraIOLimelight4(String name, Supplier<Rotation2d> rotationSupplier) {
        super(name, rotationSupplier);

        bindIMUCommands();
        bindThrottleCommands();
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        super.updateInputs(inputs);

        if (current_imu_mode_ != currentMode_.id) {
            LimelightHelpers.SetIMUMode(name_, currentMode_.id); // Set IMU Mode
            current_imu_mode_ = currentMode_.id ;
        }

        if (current_ll_throttle_ != currentThrottle_) {
            LimelightHelpers.setLimelightNTDouble(name_, "throttle_set", currentThrottle_); // Set Throttle
            current_ll_throttle_ = currentThrottle_ ;
        }

        inputs.imuMode = currentMode_;
        inputs.imuRobotYaw = Rotation2d.fromDegrees(LimelightHelpers.getIMUData(name_).robotYaw);
        inputs.frameSkipThrottle = currentThrottle_;
    }

    private void bindThrottleCommands() {
        // If not throttling, skip binding these commands.
        if (!VisionConstants.regulateThrottle) return;

        enabled.whileTrue(setThrottleCommand(VisionConstants.numSkippedFramesEnabled));
        enabled.whileFalse(setThrottleCommand(VisionConstants.numSkippedFramesDisabled));
    }

    private void bindIMUCommands() {
        // If running without IMU, skip binding these commands.
        if (!VisionConstants.useIMU) return;

        // When enabled, set to use IMU after slight offset.
        enabled.onTrue(
            setModeCommand(VisionConstants.enabledIMUMode).beforeStarting(Commands.waitTime(Milliseconds.of(100)))
        );

        // When disabled, use gryo to reset IMU.
        enabled.onFalse(setModeCommand(IMUMode.SEEDING));
    }

    private void setThrottle(int throttle) {
        currentThrottle_ = throttle;
    }

    private Command setThrottleCommand(int throttle) {
        return Commands.runOnce(() -> setThrottle(throttle)).ignoringDisable(true);
    }

    private void setMode(IMUMode mode) {
        currentMode_ = mode;
    }

    private Command setModeCommand(IMUMode mode) {
        return Commands.runOnce(() -> setMode(mode)).ignoringDisable(true);
    }

}
