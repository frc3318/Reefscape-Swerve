package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.TagApproaches;

public class Vision extends SubsystemBase {
    public AprilTagFieldLayout AprilTag_FieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public Pose2d getVisionCalculatedRobotPose = new Pose2d();

    private static final Vision m_Vision = new Vision();

    // PID values and supporting storage for turning towards targets
    private double _turnkp = .0065;
    private double _turnki = 0.0;
    private double _turnkd = 0;

    private double _rotkp = .0075;
    private double _rotki = 0.0;
    private double _rotkd = 0;

    private PIDController _turnToTargetPID = new PIDController(_turnkp, _turnki, _turnkd);
    private PIDController _rotateToTargetPID = new PIDController(_rotkp, _rotki, _rotkd);
    private double turnPower = 0;
    private double rotPower = 0;


    private String _limelightName = "limelight-tags";

    // Supplier of pose information for each pose.
    private TagApproaches _tagApproches;

    public static Vision getInstance() {
        return m_Vision;
    }

    public Vision() {
        _tagApproches = new TagApproaches();

        // Set tolerance to 2 degrees
        _turnToTargetPID.setTolerance(2);
        _rotateToTargetPID.setTolerance(2);
    }

    private Pose2d currentOptimalPose;

    @Override
    public void periodic() {
        // Periodically, update the data on the current target
        UpdateTargetData();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    private void UpdateTargetData() {
        boolean aquired = AllianceTargetAquired();
        if (aquired) {
            int targetID = (int) LimelightHelpers.getFiducialID(_limelightName);
            CalculateStearingValues(targetID);
            this.currentOptimalPose = _tagApproches.DesiredRobotPos(targetID);
        }
    }

    private Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    private void CalculateStearingValues(int targetID) {
        // using data from tag approaches, determine the desired pose for the target
        // currently be tracked
        /// TO DO - Calculate the desired pose for the target we are tracking. Do this
        // only if the target is ours.///
        /// otherwise return null??? or something to that affect.
        this.currentOptimalPose = new Pose2d(0, 0, new Rotation2d(0));

        /// other calculations for PID turning to target may be appropriate here as
        /// well.
        turnPower = _turnToTargetPID.calculate(LimelightHelpers.getTX(_limelightName), 0);
        rotPower = _rotateToTargetPID.calculate(Units.radiansToDegrees(LimelightHelpers.getBotPose3d_TargetSpace(_limelightName).getRotation().getY()),0);
        if (_turnToTargetPID.atSetpoint())
            turnPower = 0;
        
        if (_rotateToTargetPID.atSetpoint())
            rotPower = 0;
    }

    public Pose2d GetTargetPose() {
        return this.currentOptimalPose;
    }

    // public void UpdatePoseEstimatorWithVisionBotPose(SwerveDrivePoseEstimator swervePoseEstimator) {
    //     LimelightHelpers.PoseEstimate estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(_limelightName);
 
    //     if (estimatedPose.pose.getX() == 0.0) {
    //         return;
    //     }

    //     int fidID = (int) LimelightHelpers.getFiducialID(_limelightName);

    //     if ((fidID >= 0) && (fidID <= 15)) {
    //         Pose2d pose = _tagApproches.TagFieldPose2d(fidID);

    //         // calculating distance from robot to target
    //         Translation2d trans1 = new Translation2d(estimatedPose.pose.getX(), estimatedPose.pose.getY());
    //         Translation2d trans2 = new Translation2d(pose.getX(), pose.getY());
    //         double poseDifference = trans1.getDistance(trans2);

    //         // take the estimated robot pose and add PI radians to say the camera is on the back of the robot.
    //         Pose2d robotPose  = new Pose2d(
    //             estimatedPose.pose.getX(),
    //             estimatedPose.pose.getY(),
    //             new Rotation2d(estimatedPose.pose.getRotation().getRadians() + Math.PI)
    //         );
    //         // offset from cameral to middle of robot
    //         robotPose = robotPose.transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(10.5),0),new Rotation2d()));
       
 
    //         getVisionCalculatedRobotPose = robotPose;    


    //         if (poseDifference < 1.5) {
    //             if (swervePoseEstimator != null) {
    //                 swervePoseEstimator.addVisionMeasurement(robotPose,
    //                         Timer.getFPGATimestamp() - estimatedPose.latency);
                    
    //            }
    //            RobotContainer.getInstance().s_Swerve.setPose(robotPose);
    //         }
    //     }
    // }

    // returns a blank pose if no tags are available to return a pose. Otherwise
    // returns where the camera is
    // relative to the field.
    public Pose2d GetRobotLimelightPoseEstimate() {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(_limelightName);
        if (estimate.pose.getX() == 0.0) {
            return new Pose2d();
        } else {
            // return new Pose2d();
            return estimate.pose;
        }
    }

    public boolean AllianceTargetAquired() {
        boolean targetAquired = LimelightHelpers.getTV(_limelightName);
        if (targetAquired) {
            int targetID = (int) LimelightHelpers.getFiducialID(_limelightName);
            if ((targetID >= 0) && (targetID <= 16))
                return (MyAlliance() == _tagApproches.TagAlliance(targetID));
            else
                return false;
        }
        return false;
    }

    public double GetTargetTurnPower() {
        return turnPower;
    }

    public double GetTargetRotPower() {
        return -rotPower;
    }
}