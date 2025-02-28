package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ReefConstants;
import frc.robot.util.ReefAdjustments.ReefAdjustmentDistances;

public class ReefFaceInfo {
    private final static boolean kUseAdjustments = false ;

    private final ReefFace face_;

    private final Pose2d tagPose_;
    private final Pose2d algaeCollectPose_;
    private final Pose2d algaeBackupPose_;
    private final Pose2d leftScoringPose_;
    private final Pose2d rightScoringPose_;
    private final Pose2d leftScoringWithAlgaePose_;
    private final Pose2d rightScoringWithAlgaePose_;

    public ReefFaceInfo(ReefFace face) {
        face_ = face;

        tagPose_ = FieldConstants.layout.getTagPose(face.tagID_).orElseThrow().toPose2d().transformBy(
            new Transform2d(
                new Translation2d(),
                new Rotation2d(Degrees.of(180))
        ));        

        algaeCollectPose_ = tagPose_.transformBy(new Transform2d(
            new Translation2d(
                ReefConstants.distanceFromTagAlgae.unaryMinus(),
                ReefConstants.robotToArm.unaryMinus()
            ),
            new Rotation2d()
        ));      

        algaeBackupPose_ = algaeCollectPose_.transformBy(new Transform2d(
            new Translation2d(
                ReefConstants.backupDistanceAlgae.unaryMinus(),
                Meters.zero()
            ),
            new Rotation2d()
        ));        
        
        leftScoringPose_ = tagPose_.transformBy(new Transform2d(
            new Translation2d(
                getReefArmDistance(face.tagID_, true).unaryMinus(),
                getLeftRightDistance(face.tagID_, true, false).minus(ReefConstants.robotToArm)
            ),
            new Rotation2d()
        ));

        leftScoringWithAlgaePose_ = tagPose_.transformBy(new Transform2d(
            new Translation2d(
                getReefArmDistance(face.tagID_, true).unaryMinus(),
                getLeftRightDistance(face.tagID_, true, true).minus(ReefConstants.robotToArm)
            ),
            new Rotation2d()
        ));       
        
        rightScoringPose_ = tagPose_.transformBy(new Transform2d(
            new Translation2d(
                getReefArmDistance(face.tagID_, false).unaryMinus(),
                getLeftRightDistance(face.tagID_, false, false).unaryMinus().minus(ReefConstants.robotToArm)
            ),
            new Rotation2d()
        ));

        rightScoringWithAlgaePose_ = tagPose_.transformBy(new Transform2d(
            new Translation2d(
                getReefArmDistance(face.tagID_, false).unaryMinus(),
                getLeftRightDistance(face.tagID_, false, true).unaryMinus().minus(ReefConstants.robotToArm)
            ),
            new Rotation2d()
        ));        
    }

    public ReefFace getFace() {
        return face_;
    }

    public Pose2d getTagPose() {
        return tagPose_;
    }

    public Pose2d getAlgaeCollectPose() {
        return algaeCollectPose_;
    }

    public Pose2d getAlgaeBackupPose() {
        return algaeBackupPose_;
    }

    public Pose2d getLeftScoringPose() {
        return leftScoringPose_;
    }

    public Pose2d getRightScoringPose() {
        return rightScoringPose_;
    }

    public Pose2d getLeftScoringWithAlgaePose() {
        return leftScoringWithAlgaePose_;
    }

    public Pose2d getRightScoringWithAlgaePose() {
        return rightScoringWithAlgaePose_;
    }

    private Distance getLeftRightDistance(int tagid, boolean left, boolean algae) {
        Distance ret = ReefConstants.leftRightOffset ;

        if (ReefFaceInfo.kUseAdjustments) {
            ReefAdjustmentDistances dists = ReefAdjustments.AdjustmentData.adjustments.get(tagid) ;
            ret = ReefConstants.leftRightOffset.plus(left ? dists.left_right_left_.unaryMinus() : dists.left_right_right_) ;
        }

        return ret ;
    }

    private Distance getReefArmDistance(int tagid, boolean left) {
        Distance ret = ReefConstants.distanceFromTagCoral ;

        if (ReefFaceInfo.kUseAdjustments) {
            ReefAdjustmentDistances dists = ReefAdjustments.AdjustmentData.adjustments.get(tagid) ;
            ret = ReefConstants.distanceFromTagCoral.plus(left ? dists.forward_back_left_ : dists.forward_back_right_) ;
        }

        return ret;
    }    
}
