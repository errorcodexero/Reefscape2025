package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;

public enum ClimberState {
    Stowed(ClimberConstants.Climber.Position.kStowed),
    PrepareToClimb(ClimberConstants.Climber.Position.kPrepped),
    Climb(ClimberConstants.Climber.Position.kClimbed);


    private final Angle angle;

    private ClimberState(Angle angle) {
        this.angle = angle;
    }

    public Angle getAngle() {
        return angle;
    }    
}
