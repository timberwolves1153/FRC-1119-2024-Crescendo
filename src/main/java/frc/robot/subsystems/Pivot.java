package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class Pivot {
    
    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;
    private SparkPIDController pivotMotor1PID;
    private SparkPIDController pivotMotor2PID;
    private RelativeEncoder pivotMotor1Encoder;
    private RelativeEncoder pivotMotor2Encoder;
    private DigitalInput limitSwitch;


    public Pivot() {

        pivotMotor1 = new CANSparkMax(41, MotorType.kBrushless);
        pivotMotor2 = new CANSparkMax(42, MotorType.kBrushless);

        pivotMotor1.getPIDController();
        pivotMotor2.getPIDController();

        pivotMotor1.restoreFactoryDefaults();
        pivotMotor2.restoreFactoryDefaults();

        limitSwitch = new DigitalInput(1);

    }

    public void pivotUp() {
        pivotMotor1.setVoltage(4);
        pivotMotor2.setVoltage(4);
    }

    public void pivotDown() {
        pivotMotor1.setVoltage(-4);
        pivotMotor2.setVoltage(-4);
    }

    public void pivotStop() {
        pivotMotor1.setVoltage(0);
        pivotMotor2.setVoltage(0);
    }

}
