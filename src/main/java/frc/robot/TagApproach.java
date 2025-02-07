package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TagApproach {
    enum gameTarget {
        Amp,
        Speaker,
        Source, 
        Stage
    }
    
    private int _fiduciaryNumber;
    private Alliance _alliance;
    private gameTarget _targetType;
    private Pose2d _desiredPose;
    
    public TagApproach(int id, Alliance alliance, gameTarget targetType, 
        Pose2d desiredPose) {
        _fiduciaryNumber = id;
        _alliance = alliance;
        _targetType = targetType;
        _desiredPose = desiredPose;
    }

    public int FiduciaryNumber(){
        return _fiduciaryNumber;
    }

    public Alliance TagAlliance(){
        return _alliance;
    }

    public gameTarget GameTarget(){
        return _targetType;
    }
    
    public Pose2d DesiredPos(){
        return _desiredPose;
    }

}
