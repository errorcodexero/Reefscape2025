package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.Funnel;

public class WaitForCoralCmd extends Command {
    private Funnel funnel_;
    private WaitForCoralCmdState WaitForCoralCmdState_;
        private enum WaitForCoralCmdState {
            RollersOn,
            WaitForCoral,
            RollersOff,
            End
        }
    
        public WaitForCoralCmd (Funnel funnel) {
            addRequirements(funnel);
            funnel_ = funnel;
        }
        
        @Override
        public void initialize() {
            WaitForCoralCmdState_ = WaitForCoralCmdState.RollersOn;
        }

        @Override
        public void execute() {
            switch (WaitForCoralCmdState_) {
                case RollersOn:
                    if (funnel_.coralFunnelRisingEdge()) {
                        funnel_.hasSeenCoral();
                        WaitForCoralCmdState_ = WaitForCoralCmdState.RollersOff;
                    }
                    break;
                case RollersOff:
                    WaitForCoralCmdState_ = WaitForCoralCmdState.End;
                    break;
                case End:
                    break;
            }
        }
        
        public void end(boolean canceled) {
            WaitForCoralCmdState_ = WaitForCoralCmdState.End;
        }
}
