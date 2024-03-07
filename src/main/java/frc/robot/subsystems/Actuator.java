package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Actuator {
    
    private CANSparkMax actuatorMotor;

    public Actuator() {

        actuatorMotor = new CANSparkMax(43, MotorType.kBrushless);

    }

    public void climberUp() {
        actuatorMotor.setVoltage(6);
    }

    public void climberDown() {
        actuatorMotor.setVoltage(-6);
    }

    public void climberStop() {
        actuatorMotor.setVoltage(0);
    }

}