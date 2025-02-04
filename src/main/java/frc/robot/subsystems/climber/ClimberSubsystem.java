/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
 */

 package frc.robot.subsystems.climber;

 import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 public class ClimberSubsystem extends SubsystemBase{
   private ClimberIO io_; 
   private ClimberIOInputsAutoLogged inputs_ = new ClimberIOInputsAutoLogged();
   private Angle target_ ;

   public ClimberSubsystem(ClimberIO io) {
      io_ = io;
      inputs_ = new ClimberIOInputsAutoLogged();
   }
 
   @Override
   public void periodic(){
      io_.updateInputs(inputs_);
      Logger.processInputs("Climber", inputs_);
   }

   public void setClimberPosition(Angle a) {
      target_ = a ;
      io_.setClimberPosition(a);
   }

   public boolean isAtTarget() {
      return inputs_.climberPosition.isNear(target_, ClimberConstants.kPositionTolerance) && 
             inputs_.climberVelocity.isNear(DegreesPerSecond.of(0.0), ClimberConstants.kVelocityTolerance) ;
   }
}
 
 