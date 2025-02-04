/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
 */

 package frc.robot.subsystems.funnel ;

 import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 public class FunnelSubsystem extends SubsystemBase{
   private FunnelIO io_; 
   private FunnelIOInputsAutoLogged inputs_ = new FunnelIOInputsAutoLogged();
   private Angle target_ ;

   public FunnelSubsystem(FunnelIO io) {
      io_ = io;
      inputs_ = new FunnelIOInputsAutoLogged();
   }
 
   @Override
   public void periodic(){
      io_.updateInputs(inputs_);
      Logger.processInputs("Funnel", inputs_);
   }      

   public void setTargetPosition(Angle pos) {
      target_ = pos ;
      io_.setTargetPosition(pos);
   }

   public boolean isAtTarget() {
      return inputs_.funnelPosition.isNear(target_, FunnelConstants.kPositionTolerance) && 
             inputs_.funnelVelocity.isNear(DegreesPerSecond.of(0.0), FunnelConstants.kVelocityTolerance) ;
   }
}
 
 