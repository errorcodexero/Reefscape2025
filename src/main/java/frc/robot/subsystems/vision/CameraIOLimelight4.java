package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CameraIOLimelight4 extends CameraIOLimelight {

    /**
     * An enum representing the mode of the Limelight4's IMU. This is an enum for log readiblity.
     */
    public static enum IMUMode {
        NONE(0),
        IGNORING(0),
        SEEDING(1),
        USING(2);

        private final int id;

        private IMUMode(int id) {
            this.id = id;
        }
    }

    // Enabled Trigger
    private static final Trigger enabled = RobotModeTriggers.disabled().negate();

    // The current mode of the IMU, assuming this is not set anywhere else
    private IMUMode currentMode_ = VisionConstants.runWithoutIMU ? IMUMode.IGNORING : IMUMode.SEEDING;
    
    public CameraIOLimelight4(String name, Supplier<Rotation2d> rotationSupplier) {
        super(name, rotationSupplier);
        
        setMode(currentMode_);

        // If running without IMU, skip binding these commands.
        if (VisionConstants.runWithoutIMU) return;

        // When enabled, set to use IMU after slight offset.
        enabled.onTrue(Commands.sequence(
            Commands.waitTime(Milliseconds.of(100)),
            setModeCommand(IMUMode.USING)
        ));

        // When disabled, use gryo to reset IMU.
        enabled.onFalse(setModeCommand(IMUMode.SEEDING));
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        super.updateInputs(inputs);

        inputs.imuMode = currentMode_;
        inputs.imuRobotYaw = Rotation2d.fromDegrees(LimelightHelpers.getIMUData(name_).robotYaw);
    }

    private void setMode(IMUMode mode) {
        LimelightHelpers.SetIMUMode(name_, mode.id);
        currentMode_ = mode;
    }

    private Command setModeCommand(IMUMode mode) {
        return Commands.runOnce(() -> setMode(mode)).ignoringDisable(true);
    }

}
