package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
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
    private IMUMode currentMode_ = IMUMode.SEEDING;
    
    public CameraIOLimelight4(String name, Supplier<Rotation2d> rotationSupplier) {
        super(name, rotationSupplier);
        
        setMode(currentMode_);

        // When enabled, set to use IMU, when disabled, seed IMU
        enabled.onChange(Commands.runOnce(
            () -> setMode(RobotState.isDisabled() ? IMUMode.SEEDING : IMUMode.USING)
        ).ignoringDisable(true));
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        super.updateInputs(inputs);

        inputs.imuMode = currentMode_;
    }

    /**
     * Sets the mode of the IMU.
     * @param mode The mode to set it to.
     */
    private void setMode(IMUMode mode) {
        LimelightHelpers.SetIMUMode(name_, mode.id);
        currentMode_ = mode;
    }

}
