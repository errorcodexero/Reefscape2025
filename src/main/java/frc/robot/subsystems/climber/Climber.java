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
 
      public Climber(ClimberIO io){
         io_ = io;
      }
 
      @Override
      public void periodic(){
         io_.updateInputs(inputs_);
      }
 }
 
 