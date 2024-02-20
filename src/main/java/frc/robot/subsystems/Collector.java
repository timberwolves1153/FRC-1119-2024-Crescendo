package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector extends SubsystemBase {

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

    public double collectorVolatge(){
        return (collectorMotor.getBusVoltage()) * (collectorMotor.getAppliedOutput());
    }
}
