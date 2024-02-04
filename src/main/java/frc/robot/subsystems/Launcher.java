package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants;

public class Launcher implements Subsystem {
    private CANSparkMax shooterMotor;

    private SparkPIDController shooterPID;
    private PIDController shooterMotorController;

    private SimpleMotorFeedforward shooterFF;

    private SysIdRoutine shooterRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors,this)
        );

         private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0)); 
         private final MutableMeasure<Velocity<Angle>> velocity = mutable(RPM.of(0));

  public Launcher(){

    shooterMotor.restoreFactoryDefaults();

    shooterMotor = new CANSparkMax(61, MotorType.kBrushless);

    shooterMotorController = new PIDController(0, 0, 0); //TUNE IT

    shooterFF = new SimpleMotorFeedforward(0,0,0); //TUNEEEEEEE

    configMotors(); 


}


public void shootAmp(){
    shooterMotor.setVoltage(6);

}

public void shootSpeaker(){
    shooterMotor.setVoltage(6);
}

public double getVelocity() {
    return shooterMotor.getEncoder().getVelocity();
}

public void setLauncherVelocity(double setpoint){
    double feedBack = shooterMotorController.calculate(getVelocity(), setpoint);
    double feedForward = shooterFF.calculate(setpoint);
    shooterMotor.setVoltage(feedBack + feedForward);
}






// public void shootSpeakerLine(){
//     shooterMotor.setVoltage(6);
// }

// public void launcherStop() {
//     shooterMotor.setVoltage(0);
// }

// public void ejectDisc() {
//     shooterMotor.setVoltage(-6);
// }


public void configMotors(){
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.clearFaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setInverted(false);
    shooterMotor.burnFlash();
}

    private void voltageDrive(Measure<Voltage> voltage){
        shooterMotor.setVoltage(voltage.in(Volts));
    }

    private void logMotors(SysIdRoutineLog logger){
        logger.motor("pivot")
        .voltage
        (appliedVoltage.mut_replace
        (
            shooterMotor.getAppliedOutput()*shooterMotor.getBusVoltage(), Volts
            )
        ).angularVelocity(velocity.mut_replace(getVelocity(), RPM));
    }
}
