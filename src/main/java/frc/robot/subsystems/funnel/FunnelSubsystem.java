
package frc.robot.subsystems.funnel ;

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

   public Command funnelSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return funnelIdRoutine().quasistatic(dir) ;
    }

    public Command funnelSysIdDynamic(SysIdRoutine.Direction dir) {
        return funnelIdRoutine().dynamic(dir) ;
    }    

    private SysIdRoutine funnelIdRoutine() {
        Voltage step = Volts.of(7) ;
        Time to = Seconds.of(10.0) ;
        SysIdRoutine.Config cfg = new SysIdRoutine.Config(null, step, to, null) ;

        SysIdRoutine.Mechanism mfg = new SysIdRoutine.Mechanism(
                                        (volts) -> io_.setFunnelMotorVoltage(volts.magnitude()),
                                        (log) -> io_.logFunnelMotor(log),
                                        this) ;

        return  new SysIdRoutine(cfg, mfg) ;
    }    
}
 
 