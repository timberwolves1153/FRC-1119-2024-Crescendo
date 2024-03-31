package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase {
    
    private CANSparkMax actuatorMotor;
    //DigitalInput actuatorLimit = new DigitalInput(2);

    public Actuator() {

        actuatorMotor = new CANSparkMax(43, MotorType.kBrushless);

    }

   
    // public boolean actuatorLimit() {
    //     return !actuatorLimit.get();
    // }

    // public void climberDown() {
    //     if (actuatorLimit()) {
    //         actuatorMotor.setVoltage(0);
    //     }
    //     else {
    //         actuatorMotor.setVoltage(10);
    //     }

    // }
 
    public void climberUp() {
        actuatorMotor.setVoltage(-10);
    }

    public void climberStop() {
        actuatorMotor.setVoltage(0);
    }

   

}