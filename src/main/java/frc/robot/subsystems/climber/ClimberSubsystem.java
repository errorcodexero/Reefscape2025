/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
*/

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ClimberSubsystem extends SubsystemBase{
   private ClimberIO io_; 
   private ClimberIOInputsAutoLogged inputs_ = new ClimberIOInputsAutoLogged();
   private Angle target_angle_ ;

    public ClimberSubsystem(ClimberIO io) {
        io_ = io; 
        inputs_ = new ClimberIOInputsAutoLogged(); 
    }

//io_.moveClimber(Degrees.of(180));

   public void setClimberPosition(Angle angle) {
      target_angle_ = angle;
      io_.setClimberPosition(angle);
   }

   public boolean isClimberAtTarget() {
        if((inputs_.climberPosition.isNear(target_angle_, ClimberConstants.Climber.kPosTolerance)) && (inputs_.climberVelocity.isNear(DegreesPerSecond.of(0), ClimberConstants.Climber.kVelTolerance))) {
            return true; 
        }
        return false; 
    }

   @Override
   public void periodic(){
      io_.updateInputs(inputs_);
   }
}
 
 