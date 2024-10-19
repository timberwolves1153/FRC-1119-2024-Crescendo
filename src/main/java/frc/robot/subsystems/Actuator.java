package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase {
    
    private CANSparkMax actuatorMotor;
    DigitalInput topLimit = new DigitalInput(4);
    DigitalInput bottomLimit = new DigitalInput(3);

    public Actuator() {

        actuatorMotor = new CANSparkMax(43, MotorType.kBrushless);

    }

   
    public boolean actuatorLimitUp() {
        return !topLimit.get();
    }

    public boolean actuatorLimitDown() {
        return !bottomLimit.get();
    }

    public void climberDown() {
      //  actuatorMotor.setVoltage(10);
        if (actuatorLimitDown()) {
            actuatorMotor.setVoltage(0);
        } else {
            actuatorMotor.setVoltage(10);
        }
    }
 
    public void climberUp() {
        actuatorMotor.setVoltage(-10);
        if (actuatorLimitUp()) {
            actuatorMotor.setVoltage(0);
        }
        else {
            actuatorMotor.setVoltage(-10);
        }
    }

    public void climberStop() {
        actuatorMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bottom Limit Switch", actuatorLimitDown());
        SmartDashboard.putBoolean("Upper Limit Switch", actuatorLimitUp());
    }

   

}