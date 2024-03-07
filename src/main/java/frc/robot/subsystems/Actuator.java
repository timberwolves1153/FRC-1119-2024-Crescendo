package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase {
    
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