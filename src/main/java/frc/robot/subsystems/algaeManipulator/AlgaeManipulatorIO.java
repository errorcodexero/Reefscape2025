package frc.robot.subsystems.algaeManipulator;

import org.littletonrobotics.junction.AutoLog;

public abstract interface AlgaeManipulatorIO{
    @AutoLog
    public class AlgaeManipulatorIOInputs{
        
    }

    abstract void update(AlgaeManipulatorIOInputsAutoLogged inputs_);
}
