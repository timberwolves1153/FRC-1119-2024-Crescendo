package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector extends SubsystemBase {

    private CANSparkMax collectorMotor;
    private DigitalInput limit;

    public Collector() {
        
        collectorMotor = new CANSparkMax(51, MotorType.kBrushless);
        limit = new DigitalInput(2);
        collectorMotor.setSmartCurrentLimit(40);  

    }
    public boolean hasNote() {
        return !limit.get();
    }

    public void collectorIntake() { 
        collectorMotor.setVoltage(-10);
    }

    public void intakeOverride() {
        collectorMotor.setVoltage(-10);
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

    @Override

    public void periodic() {
        SmartDashboard.putBoolean("Has Note", hasNote());
    }
}
