package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.util.LimelightHelpers;

public class Limelight {
    
    public double targetAngularVelocity;
    private double kP = 0.35;

    double limelight_aim_proportional(){

      double kP = .000;//TUNE

      double targetAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

      targetAngularVelocity *= Constants.maxAngularVelocity;

      targetAngularVelocity *= -1.0;

      return targetAngularVelocity;





    

    }

    double limelight_range_proportional(){
        double kP = .000;//TUNE
        double targetForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetForwardSpeed *= Swerve.kMaxAngularSpeed;
        targetForwardSpeed *= -1.0;
        return targetForwardSpeed;
    }

    private void drive(boolean fieldRelative){

        var xSpeed =
            -m_xspeedLimiter.calulate()

    }
}


