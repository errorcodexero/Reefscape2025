/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
 */

 package frc.robot.subsystems.climber;

 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 public class Climber extends SubsystemBase{
   private ClimberIO io_; 
   private ClimberIOInputsAutoLogged inputs_ = new ClimberIOInputsAutoLogged();

   enum ClimberState{
      Idle,
      DeployClimber,
      WaitToHook,
      ExecuteClimb
   }
   ClimberState climberState_;
 
   public Climber(ClimberIO io){
      io_ = io;
      climberState_ = ClimberState.Idle;
   }
 
   @Override
   public void periodic(){
      io_.updateInputs(inputs_);
   }
   public void climber() {
      switch(climberState_){
         case Idle:
            // 1. 
            // 2. 
            break;
         case DeployClimber:
            break;
         case WaitToHook:
            break;
         case ExecuteClimb:
            break;
      }
   }
      
 }
 
 