package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{

    double limelight_aim_proportional() {
        double kP = 0.0; //TUNE
    
   // double targetOffsetAngle = Limelight.getTX("limelight");
    double targetingAngularVelocity = Limelight.getTX("limelight") * kP;

    targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
    }   

    double limelight_range_proportional()
  {    
    double kP = 0.0; //TUNE 
    //double dashboardTargetingForwardSpeed = Limelight.getTY("limelight");
    double targetingForwardSpeed = Limelight.getTY("limelight") * kP;
    targetingForwardSpeed *= Constants.Swerve.maxSpeed;
    targetingForwardSpeed *= -1.0;        

  //  SmartDashboard.putNumber("LimelightSpeed", dashboardTargetingForwardSpeed);

    return targetingForwardSpeed;

  }

    public AprilTagFieldLayout aprilTagFieldLayout;
    public Limelight limelight;

    public final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(9.75); //CHANGE BASED ON CAMERA PLACEMENT
    public final double TARGET_HEIGHT_METERS = Units.inchesToMeters(57);
    public final double LIMELIGHT_MOUNT_DEGREES = Units.degreesToRadians(61);
    public final double LIMELIGHT_CALCULATED_DEGREES = Units.degreesToRadians(limelight.getTY("limelight"));

    public Vision() {

        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    }

    public double getDistance() {
        final double tan = Math.tan(LIMELIGHT_MOUNT_DEGREES + LIMELIGHT_CALCULATED_DEGREES);
        return (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT_METERS) / tan;   
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Limelight Distance", getDistance());

    }

}
