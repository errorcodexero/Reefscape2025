/*This is the ClimberSubsystem class. It is going to be used to manage 
what the climber is going to do on the robot. The climber is what the
robot is going to use to go up the deep cage. This code is incomplete

Notes on how Climber works (Fork and Spaghetti method): 
Robot drives into cage at slight angle, 2 hooks grab onto one side of cage, 
rotates climber mechanism into the robot such that the chain is pointing into
the robot, allowing the robot to essentially hover over the ground.
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
 
 