package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector extends SubsystemBase {

    private CANSparkMax collectorMotor;
    DigitalInput limit = new DigitalInput(2);

    public Collector() {
        
        collectorMotor = new CANSparkMax(51, MotorType.kBrushless);

        collectorMotor.setSmartCurrentLimit(40);  

    }

    public void collectorIntake() { 
        if(limit.get()) {
            collectorMotor.setVoltage(0);
        } else if(!limit.get()) {
            collectorMotor.setVoltage(-8);
        } else {
            collectorMotor.setVoltage(0);
        }
    }

    public void intakeOverride() {
        collectorMotor.setVoltage(-8);
    }

    public void collectorOuttake() {
        collectorMotor.setVoltage(3);
    }
    
    public void collectorStop() {
        collectorMotor.setVoltage(0);
    }

    public double collectorVolatge(){
        return (collectorMotor.getBusVoltage()) * (collectorMotor.getAppliedOutput());
    }
}
