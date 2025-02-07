/*Fork and spaghetti - Drive into cage, hooks hook onto bar, climber twists chain and cage into robot, 
so hover off ground. 
Commands - DeployClimber, ExecuteClimb
States(enum) - IDLE, DeployClimber, WaitToHook, Climb
 */

 package frc.robot.subsystems.climber;

 import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
 
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

    public Command climberSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return climberIdRoutine().quasistatic(dir) ;
    }

    public Command climberSysIdDynamic(SysIdRoutine.Direction dir) {
        return climberIdRoutine().dynamic(dir) ;
    }    

    private SysIdRoutine climberIdRoutine() {
        Voltage step = Volts.of(7) ;
        Time to = Seconds.of(10.0) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setClimberMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logClimberMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }      
}
 
 