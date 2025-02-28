package frc.robot.util;

public enum ReefFace {
    RED_AB(7),
    RED_CD(8),
    RED_EF(9),
    RED_GH(10),
    RED_IJ(11),
    RED_KL(6),
    BLUE_AB(18),
    BLUE_CD(17),
    BLUE_EF(22),
    BLUE_GH(21),
    BLUE_IJ(20),
    BLUE_KL(19);

    public final int tagID_;

    private ReefFace(int aprilTagID) {
        tagID_ = aprilTagID;
    }
}
