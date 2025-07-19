package frc.robot.subsystems.grabber.commands;


import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.oi.OISubsystem;

public class WaitForCoralCmd extends Command {

    private static boolean showState = true ;

    private OISubsystem oi_ ;
    private GrabberSubsystem grabber_;
    private FunnelSubsystem funnel_ ;
    private State state_;
    private boolean rumbling_ ;

    private enum State {
        WaitingForCoral,
        WaitForTrailingEdge,
        Finish
    }

    public WaitForCoralCmd(OISubsystem oi, FunnelSubsystem funnel, GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
        funnel_ = funnel ;
        oi_ = oi ;
    }

    @Override
    public void initialize() {
        grabber_.setGrabberMotorVoltage(GrabberConstants.kCollectVoltage) ;
        state_ = State.WaitForTrailingEdge ;
        rumbling_ = false ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_) {
            case WaitingForCoral:
                if (oi_ != null && funnel_.lowerCoralSensor() && !rumbling_) {
                    oi_.rumble(Seconds.of(1.0));
                    rumbling_ = true ;
                }

                if (grabber_.coralSensor()) {
                    grabber_.setGrabberMotorVoltage(Volts.zero()) ;
                    state_ = State.Finish;
                }
                else if (funnel_.lowerCoralSensor()) {
                    state_ = State.WaitForTrailingEdge;
                }
                break;

            case WaitForTrailingEdge:
                if (grabber_.coralSensor()) {
                    grabber_.setGrabberMotorVoltage(Volts.zero()) ;
                    grabber_.collecting();
                    state_ = State.Finish;
                }
                else if (!funnel_.lowerCoralSensor()) {
                    state_ = State.WaitForTrailingEdge;
                    grabber_.collecting();
                    state_ = State.Finish ;
                }
                break ;

            case Finish:
                break;
        }

        if (showState) {
            Logger.recordOutput("waitforcoral", state_.toString());
        }
    }

    @Override
    public void end(boolean canceled) {
        state_ = State.Finish;
    }
}
