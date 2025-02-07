
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.TagApproaches;
import frc.robot.subsystems.Swerve;

/**
 *
 */
public class LineUpToTag extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private String _limelightName = "limelight-tags"; // use from vision TODO
    public AprilTagFieldLayout FieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final Swerve m_swerve;
    private Pose2d goalPose;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, .1, OMEGA_CONSTRAINTS);

    private int lastTarget;

    public LineUpToTag(Swerve subsystem, Supplier<Pose2d> poseProvider) {
        m_swerve = subsystem;
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(m_swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set position of camera based on target seen 
        lastTarget = 0;
        // Verify that see a valid target for aliance and set current robot pose based on it.
          if (RobotContainer.getInstance().m_vision.AllianceTargetAquired()) {
            int fidID = (int) LimelightHelpers.getFiducialID(_limelightName);
            if ((fidID >= 0) && (fidID <= 16)) {
                var robotPose2d =  LimelightHelpers.getBotPose2d_wpiBlue(_limelightName); 
                Pose2d newPose  = new Pose2d(
                    robotPose2d.getX(),
                    robotPose2d.getY(),
                    new Rotation2d(robotPose2d.getRotation().getRadians() + Math.PI)
                );
                // offset from cameral to middle of robot
                newPose = newPose.transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(10.5),0),new Rotation2d()));
                m_swerve.setPose(newPose);
                lastTarget = fidID;
                goalPose = TagApproaches.getInstance().DesiredRobotPos(lastTarget);

                SmartDashboard.putString("current pose", newPose.toString());
                SmartDashboard.putString("goal pose", goalPose.toString());
            }
        }

        omegaController.reset(m_swerve.getPose().getRotation().getRadians());
        xController.reset(m_swerve.getPose().getX());
        yController.reset(m_swerve.getPose().getY());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (lastTarget == 0) {
            m_swerve.stop();
        } else {

            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());

            // Drive to the target
            var xSpeed = xController.calculate(m_swerve.getPose().getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(m_swerve.getPose().getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(m_swerve.getPose().getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }
            m_swerve.driveRobotRelative(
                    // new ChassisSpeeds(xSpeed, 0, 0));
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
                            m_swerve.getPose().getRotation()));
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
