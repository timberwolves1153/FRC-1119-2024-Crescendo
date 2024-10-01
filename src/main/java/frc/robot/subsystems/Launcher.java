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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants;


public class Launcher extends SubsystemBase {
    private CANSparkMax bottomShooterMotor;
    private CANSparkMax topShooterMotor;

    private SparkPIDController shooterPID;
    private PIDController shooterMotorController;

    private SimpleMotorFeedforward shooterFF;

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0)); 
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RPM.of(0));
   
    private final SysIdRoutine launcherRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        bottomShooterMotor.setVoltage(volts.in(Volts));
        topShooterMotor.setVoltage(volts.in(Volts));
      
     },
     log -> {
        log.motor("left launcher")
        .voltage(appliedVoltage.mut_replace(bottomShooterMotor.getAppliedOutput() * bottomShooterMotor.getBusVoltage(), 
        Volts)).angularVelocity(velocity.mut_replace(getLeftVelocity(), RPM));

        log.motor("right launcher")
        .voltage(appliedVoltage.mut_replace(topShooterMotor.getAppliedOutput() * topShooterMotor.getBusVoltage(), 
        Volts)).angularVelocity(velocity.mut_replace(getRightVelocity(), RPM));
     }, this));
         

  public Launcher(){

    
    bottomShooterMotor = new CANSparkMax(61, MotorType.kBrushless);
    topShooterMotor = new CANSparkMax(62, MotorType.kBrushless);

    shooterMotorController = new PIDController(0, 0, 0); //TUNE IT

    shooterFF = new SimpleMotorFeedforward(0,0,0); //TUNEEEEEEE

    configMotors(); 

}


public void shootAmp(){
    bottomShooterMotor.setVoltage(-2);
    topShooterMotor.setVoltage(-2);
}

public void shootSpeaker(){
    bottomShooterMotor.setVoltage(-8);
    topShooterMotor.setVoltage(-8);
    
}

public void shootSpeakerDistance() {
    bottomShooterMotor.setVoltage(-10);
    topShooterMotor.setVoltage(-10);
}

public void launcherStop() {
    bottomShooterMotor.setVoltage(0);
    topShooterMotor.setVoltage(0);
}

public double getLeftVelocity() {
    return bottomShooterMotor.getEncoder().getVelocity();
}

public double getRightVelocity() {
    return topShooterMotor.getEncoder().getVelocity();
}

public void setLauncherVelocity(double setpoint){
    double feedBackLeft = shooterMotorController.calculate(getLeftVelocity(), setpoint);
    double feedBackRight = shooterMotorController.calculate(getRightVelocity(), setpoint);
    double feedForward = shooterFF.calculate(setpoint);
    bottomShooterMotor.setVoltage(feedBackLeft + feedForward);
    topShooterMotor.setVoltage(feedBackRight + feedForward);
}

public double getLeftVoltage(){
    return (bottomShooterMotor.getBusVoltage()) * (bottomShooterMotor.getAppliedOutput());
}

public double getRightVoltage(){
    return (topShooterMotor.getBusVoltage()) * (topShooterMotor.getAppliedOutput());
}

public void configMotors(){
    bottomShooterMotor.restoreFactoryDefaults();
    topShooterMotor.restoreFactoryDefaults();

    bottomShooterMotor.clearFaults();
    topShooterMotor.clearFaults();

    bottomShooterMotor.setSmartCurrentLimit(40);
    topShooterMotor.setSmartCurrentLimit(40);

    bottomShooterMotor.setIdleMode(IdleMode.kBrake);
    topShooterMotor.setIdleMode(IdleMode.kBrake);

    topShooterMotor.setInverted(false);
    bottomShooterMotor.setInverted(false);

    bottomShooterMotor.burnFlash();
    topShooterMotor.burnFlash();
}

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return launcherRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return launcherRoutine.dynamic(direction);
    }

@Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Lancher Right Voltage", getRightVoltage());
    }
}
