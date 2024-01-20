package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivot {

    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    private DigitalInput lowerLimitSwitch;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController leftPivotMotorPID;
    private SparkPIDController rightPivotMotorPID;

    public double kP, kFF;

    private final double collectSetpoint = 0.0;
    private final double shootSpeakerSetpoint = 0.0;
    private final double shootAmpSetpoint = 0.0;

    public Pivot() {

       leftPivotMotor = new CANSparkMax(41, MotorType.kBrushless);
       rightPivotMotor = new CANSparkMax(42, MotorType.kBrushless);

       lowerLimitSwitch = new DigitalInput(1);

       leftPivotMotorPID = leftPivotMotor.getPIDController();
       rightPivotMotorPID = rightPivotMotor.getPIDController();
       
       kP = 0.1;
       kFF = 0.0;

       SmartDashboard.putNumber("Pivot P Value", kP);
       SmartDashboard.putNumber("Pivot Feed Forward", kFF);
       SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());


       configPivot();

    }

    public void pivotForward() {
        leftPivotMotor.setVoltage(6);
        rightPivotMotor.setVoltage(6);
    }

    public void pivotBackward() {
        leftPivotMotor.setVoltage(-6);
        rightPivotMotor.setVoltage(-6);
    }

    public void pivotStop() {
        leftPivotMotor.setVoltage(0);
        rightPivotMotor.setVoltage(0);
    }

    public double getPivotEncoder() {
        return pivotEncoder.getPosition();
    }

    public void resetPivotEncoder() {
        pivotEncoder.setPosition(0);
    }

    public void collectPosition() {
        leftPivotMotorPID.setReference(collectSetpoint, ControlType.kPosition);
     //   rightPivotMotorPID.setReference(collectSetpoint, ControlType.kPosition);
    }

    public void shootSpeakerPosition() {
        leftPivotMotorPID.setReference(collectSetpoint - 1, ControlType.kPosition); //BEFORE DRIVING, TEST THIS OUT TO MAKE SURE IT IS CORRECT
     //   rightPivotMotorPID.setReference(collectSetpoint - 1, ControlType.kPosition); //BEFORE DRIVING, TEST THIS OUT TO MAKE SURE IT IS CORRECT
    }

    public void shootAmpPosition() {
        leftPivotMotorPID.setReference(collectSetpoint - 5, ControlType.kPosition); //BEFORE DRIVING, TEST THIS OUT TO MAKE SURE IT IS CORRECT
      //  rightPivotMotorPID.setReference(collectSetpoint - 5, ControlType.kPosition); //BEFORE DRIVING, TEST THIS OUT TO MAKE SURE IT IS CORRECT
    }

    public void configPivot() {
        leftPivotMotor.restoreFactoryDefaults();
        leftPivotMotor.setInverted(false);
        leftPivotMotor.setIdleMode(IdleMode.kCoast);
        leftPivotMotor.burnFlash();

        rightPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.follow(leftPivotMotor, true);
        rightPivotMotor.setIdleMode(IdleMode.kCoast);
        rightPivotMotor.burnFlash();
    }

}
