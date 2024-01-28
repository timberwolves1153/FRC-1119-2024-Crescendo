package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher {
    private CANSparkMax shooterMotor;

  public Launcher(){

    shooterMotor.restoreFactoryDefaults();

    shooterMotor = new CANSparkMax(61, MotorType.kBrushless);

}


public void shootAmp(){
    shooterMotor.setVoltage(6);

}

public void shootSpeaker(){
    shooterMotor.setVoltage(11);
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
    shooterMotor.setVoltage(11);
}

public void launcherStop() {
    shooterMotor.setVoltage(0);
}

public void ejectDisc() {
    shooterMotor.setVoltage(-6);
}

}
