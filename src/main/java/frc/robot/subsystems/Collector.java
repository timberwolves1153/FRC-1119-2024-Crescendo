package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector {

    private CANSparkMax collectorMotor;

    public Collector() {
        
        collectorMotor = new CANSparkMax(51, MotorType.kBrushless);

        collectorMotor.setSmartCurrentLimit(40);
    }

    public void collectorIntake() {
        collectorMotor.setVoltage(-6);
    }
    public void collectorOuttake() {
        collectorMotor.setVoltage(6);
    }
    public void collectorStop() {
        collectorMotor.setVoltage(0);
    }
}