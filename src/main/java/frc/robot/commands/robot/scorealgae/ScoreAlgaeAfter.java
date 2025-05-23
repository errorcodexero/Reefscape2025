package frc.robot.commands.robot.scorealgae;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ProcessorConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.CommandConstants;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class ScoreAlgaeAfter extends XeroSequenceCmd {
    private static final int kRedAprilTag = 3;
    private static final int kBlueAprilTag = 16;

    private ManipulatorSubsystem m_;
    private GrabberSubsystem g_;
    private BrainSubsystem brain_;
    private Drive db_;
    private boolean quick_ ;

    public final static Pose2d kProcessorRedPose = new Pose2d();
    public final static Pose2d kProcessorBluePose = new Pose2d();

    public ScoreAlgaeAfter(Drive db, BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g, boolean quick) {
        super("ScoreAlgaeAfter");
        m_ = m;
        g_ = g;
        brain_ = b;
        db_ = db;
        quick_ = quick ;
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {

        Optional<Pose2d> scoringPose = getProcessorScorePose();
        if (scoringPose.isEmpty())
            return;

        seq.addCommands(
                RobotContainer.getInstance().gamepad().setLockCommand(true),
                Commands.parallel(
                        new GoToCmdDirect(m_, ManipulatorConstants.Elevator.Positions.kScoreAlgaeReef,
                                ManipulatorConstants.Arm.Positions.kScoreAlgaeReef),
                        DriveCommands.simplePathCommand(db_, scoringPose.get(),
                                CommandConstants.AlgaeScore.kMaxDriveVelocity,
                                CommandConstants.AlgaeScore.kMaxDriveAcceleration)),
                new DepositAlgaeCmd(g_),

                new SetHoldingCmd(brain_, GamePiece.NONE)) ;

                if (!quick_) {
                    seq.addCommands(
                        db_.runVelocityCmd(MetersPerSecond.of(-1.0), MetersPerSecond.of(0), RadiansPerSecond.zero()).withTimeout(Seconds.of(0.5)),
                        db_.stopCmd(),
                        new GoToCmdDirect(m_, ManipulatorConstants.Elevator.Positions.kStow,m_.getArmPosition()),
                        new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow));
                }

                seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false));
    }

    private Optional<Pose2d> getProcessorScorePose() {
        Optional<Pose2d> ret = Optional.empty();

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {

            // Get the april tag pose
            Pose2d p = FieldConstants.layout.getTagPose(alliance.get() == Alliance.Red ? kRedAprilTag : kBlueAprilTag).orElseThrow().toPose2d() ;

            Pose2d p180 = p.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0))) ;

            // Translate by the X and Y distance from the processor tag to place
            Transform2d xform = new Transform2d(
                        new Translation2d(ProcessorConstants.kXdistanceFromProcessorTag.unaryMinus(), 
                                          ProcessorConstants.kYdistanceFromProcessorTag), 
                        new Rotation2d()) ;
            Pose2d retpose = p180.transformBy(xform) ;

            ret = Optional.of(retpose) ;
        }

        return ret;
    }
}
