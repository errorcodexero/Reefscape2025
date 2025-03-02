/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
*/

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimberSubsystem extends SubsystemBase {
   private ClimberIO io_;
   private ClimberIOInputsAutoLogged inputs_ = new ClimberIOInputsAutoLogged();
   private Angle target_angle_;
   private ClimberState state_;
   private Trigger ready_to_climb_ ;

   public ClimberSubsystem(ClimberIO io) {
      io_ = io;
      inputs_ = new ClimberIOInputsAutoLogged();
      state_ = ClimberState.Stowed;

      ready_to_climb_ = new Trigger(this::readyToClimb) ;
   }

   public ClimberState getClimberState() {
      return state_;
   }

   public boolean readyToClimb() {
      return state_ == ClimberState.PrepareToClimb && isClimberAttached(); 
   }

   public Trigger readyToClimbTrigger() {
      return ready_to_climb_;
   }

   public void setClimberTarget(ClimberState state) {
      state_ = state ;
      target_angle_ = state.getAngle();
      io_.setClimberPosition(target_angle_);
   }

   public boolean isClimberAtTarget() {
      if ((inputs_.climberPosition.isNear(target_angle_, ClimberConstants.Climber.kPosTolerance))
            && (inputs_.climberVelocity.isNear(DegreesPerSecond.of(0), ClimberConstants.Climber.kVelTolerance))) {
         return true;
      }
      return false;
   }

   public boolean isClimberAttached() {
      return inputs_.attachedSensor;
   }

   public void setMotorVoltage(Voltage v) {
      io_.setClimberVoltage(v);
   }

   public Angle getClimberPosition() {
      return inputs_.climberPosition;
   }

   @Override
   public void periodic() {
      io_.updateInputs(inputs_);
      Logger.processInputs("Climber", inputs_);
   }
}
