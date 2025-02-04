/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
*/

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

 
public class Climber extends SubsystemBase{
   private ClimberIO io_; 
   private ClimberIOInputsAutoLogged inputs_ = new ClimberIOInputsAutoLogged();

   enum ClimberState{
      Idle,
      DeployClimberState,
      WaitToHookState,
      ExecuteClimbState
   }
   ClimberState climberState_;
 
   public Climber(ClimberIO io){
      io_ = io;
      climberState_ = ClimberState.Idle;
   }

   public void Idle() {
      //if oi button pressed
      climberState_ = ClimberState.DeployClimberState;
   }

   public void deployClimber() {
      io_.moveClimber(Degrees.of(90));
      climberState_ = ClimberState.WaitToHookState;
   }

   public void waitToHook() {
      //if oi button pressed
      climberState_ = ClimberState.ExecuteClimbState;
   }

   public void executeClimb() {
      io_.moveClimber(Degrees.of(180));
      climberState_ = ClimberState.Idle;
   }
 
   public void climber() {
      switch(climberState_){
         case Idle:
            break;
         case DeployClimberState:
            deployClimber();
            break;
         case WaitToHookState:
            break;
         case ExecuteClimbState:
            executeClimb();
            break;
      }
   }
   @Override
   public void periodic(){
      io_.updateInputs(inputs_);
   }
}
 
 