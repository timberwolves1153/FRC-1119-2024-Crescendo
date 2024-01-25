package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher {
    private int TBD; //THIS IS JUST A PLACEHOLDER UNTIL WE HAVE A ROBOT TO WORK WITH, DELETE WHEN SETPOINT DONE
    private CANSparkMax shooterMotor;
    private SparkPIDController shooterMotorPID;
    private RelativeEncoder shooterMotorEncoder;

    
    public double kp;
    public double shooterMotorkP;

public Launcher(){

    shooterMotor.restoreFactoryDefaults();

    shooterMotor = new CANSparkMax(61, MotorType.kBrushless);
    
    shooterMotorPID = shooterMotor.getPIDController();

    shooterMotorEncoder = shooterMotor.getEncoder();

    shooterMotorkP = 0.01;

    shooterMotorPID.setP(shooterMotorkP);

    SmartDashboard.putNumber("shooter P gain", shooterMotorkP);

    SmartDashboard.putNumber("shooter encoder", getTopRPM());


}


public void shootAmp(){
    shooterMotor.setVoltage(TBD);
}

public void shootSpeaker(){
    shooterMotor.setVoltage(TBD);
}

// public void shootSpeakerLeft(){
//     int TBD = 0; //THIS IS JUST A PLACEHOLDER UNTIL WE HAVE A ROBOT TO WORK WITH, DELETE WHEN SETPOINT DONE

//     shooterMotor.setVoltage(TBD);
// }

// public void shootSpeakerRight(){
//     int TBD = 0; //THIS IS JUST A PLACEHOLDER UNTIL WE HAVE A ROBOT TO WORK WITH, DELETE WHEN SETPOINT DONE

//     shooterMotor.setVoltage(TBD);
// }

public void shootSpeakerLine(){
    shooterMotor.setVoltage(TBD);
}

public double getTopRPM() {
    return shooterMotorEncoder.getVelocity();
}

}
