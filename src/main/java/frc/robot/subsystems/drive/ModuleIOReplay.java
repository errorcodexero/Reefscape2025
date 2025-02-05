package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * Blank Replay Implementation That Does Nothing.
 * 
 * This exists because the constructor of the ModuleIO must fit a certain signature, so I created this.
 */
public class ModuleIOReplay implements ModuleIO {
    public ModuleIOReplay(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {}
}
