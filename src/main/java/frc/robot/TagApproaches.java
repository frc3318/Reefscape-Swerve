package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TagApproach.gameTarget;

public class TagApproaches {
    public AprilTagFieldLayout FieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private TagApproach[] tagArray;

    private static TagApproaches _TagApproaches = new TagApproaches();

    public static TagApproaches getInstance() {
        return _TagApproaches;
    }

    // distance in meters from camera lens to front edge of bumper.
    public final double cameraOffset = 15;
    private double rw = 0.451; // Robot circumference in meters
    Pose2d pose;

    public TagApproaches() {
        tagArray = new TagApproach[16];

        double poseOffsetx = (FieldLayout.getTagPose(2).get().getX() - FieldLayout.getTagPose(1).get().getX()) / 2
                - 0.235;
        double poseOffsety = (FieldLayout.getTagPose(2).get().getY() - FieldLayout.getTagPose(1).get().getY()) / 2
                + 0.2;
        pose = calcNewPose(1, poseOffsetx, poseOffsety, 120);
        tagArray[0] = new TagApproach(1, Alliance.Blue, gameTarget.Source, pose);
        tagArray[1] = new TagApproach(2, Alliance.Blue, gameTarget.Source, pose);
        
        pose = calcNewPose(3, -0.79, -0.51, 240);
        tagArray[2] = new TagApproach(3, Alliance.Red, gameTarget.Speaker, pose);

        pose = calcNewPose(4, -0.914 - rw, -0.15, 180);
        tagArray[3] = new TagApproach(4, Alliance.Red, gameTarget.Speaker, pose);

        pose = calcNewPose(5, 0, -rw, -90);
        tagArray[4] = new TagApproach(5, Alliance.Red, gameTarget.Amp, pose);

        pose = calcNewPose(6, 0.0, -rw, -90);
        tagArray[5] = new TagApproach(6, Alliance.Blue, gameTarget.Amp, pose);

        pose = calcNewPose(7, 0.914 + rw, 0, 0);
        tagArray[6] = new TagApproach(7, Alliance.Blue, gameTarget.Speaker, pose);

        pose = calcNewPose(8, 0.800, -0.50, -58);
        tagArray[7] = new TagApproach(8, Alliance.Blue, gameTarget.Speaker, pose);

        poseOffsetx = (FieldLayout.getTagPose(10).get().getX() - FieldLayout.getTagPose(9).get().getX()) / 2 + 0.22;
        poseOffsety = (FieldLayout.getTagPose(10).get().getY() - FieldLayout.getTagPose(9).get().getY()) / 2 + 0.175;
        pose = calcNewPose(9, poseOffsetx, poseOffsety, 60);
        tagArray[8] = new TagApproach(9, Alliance.Red, gameTarget.Source, pose);
        tagArray[9] = new TagApproach(10, Alliance.Red, gameTarget.Source, pose);

        pose = calcNewPose(11, 0.16, -0.16, 120);
        tagArray[10] = new TagApproach(11, Alliance.Red, gameTarget.Stage, pose);

        pose = calcNewPose(12, 0.15, 0.15, 240);
        tagArray[11] = new TagApproach(12, Alliance.Red, gameTarget.Stage, pose);

        pose = calcNewPose(13, -0.3, 0, 0);
        tagArray[12] = new TagApproach(13, Alliance.Red, gameTarget.Stage, pose);

        pose = calcNewPose(14, 0.4, 0, 180);
        tagArray[13] = new TagApproach(14, Alliance.Blue, gameTarget.Stage, pose);

        pose = calcNewPose(15, -0.15, 0.15, -60);
        tagArray[14] = new TagApproach(15, Alliance.Blue, gameTarget.Stage, pose);

        pose = calcNewPose(16, -0.15, -0.15, 60);
        tagArray[15] = new TagApproach(16, Alliance.Blue, gameTarget.Stage, pose);
    }

    private Pose2d calcNewPose(int id, double arbX, double arbY, double arbAngle) {
        Pose2d tagPose = FieldLayout.getTagPose(id).get().toPose2d();

        return new Pose2d(tagPose.getX() + arbX,
                tagPose.getY() + arbY,
                new Rotation2d(Math.toRadians(arbAngle)));
    }

    public int FiduciaryNumber(int tagID) {
        return tagArray[tagID - 1].FiduciaryNumber();
    }

    public Alliance TagAlliance(int tagID) {
        return tagArray[tagID - 1].TagAlliance();
    }

    public gameTarget GameTarget(int tagID) {
        return tagArray[tagID - 1].GameTarget();
    }

    public Pose2d DesiredRobotPos(int tagID) {
        return tagArray[tagID - 1].DesiredPos();
    }

    public Pose2d TagFieldPose2d(int tagID) {
        return FieldLayout.getTagPose(tagID).get().toPose2d();
    }

}