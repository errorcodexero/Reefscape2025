package frc.robot.subsystems.CoralPlacer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class CoralPlacerSubsystem extends SubsystemBase{
    private final CoralPlacerIO io_; 
    private final CoralPlacerIOInputsAutoLogged inputs_; 

    public CoralPlacerSubsystem(CoralPlacerIO io){
        io_ = io; 
        inputs_ = new CoralPlacerIOInputsAutoLogged(); 
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
    }
}
