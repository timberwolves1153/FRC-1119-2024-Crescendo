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
    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;

    private SparkPIDController shooterPID;
    private PIDController shooterMotorController;

    private SimpleMotorFeedforward shooterFF;

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0)); 
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RPM.of(0));
   
    private final SysIdRoutine launcherRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        leftShooterMotor.setVoltage(volts.in(Volts));
        rightShooterMotor.setVoltage(volts.in(Volts));
      
     },
     log -> {
        log.motor("left launcher")
        .voltage(appliedVoltage.mut_replace(leftShooterMotor.getAppliedOutput() * leftShooterMotor.getBusVoltage(), 
        Volts)).angularVelocity(velocity.mut_replace(getLeftVelocity(), RPM));

        log.motor("right launcher")
        .voltage(appliedVoltage.mut_replace(rightShooterMotor.getAppliedOutput() * rightShooterMotor.getBusVoltage(), 
        Volts)).angularVelocity(velocity.mut_replace(getRightVelocity(), RPM));
     }, this));
         

  public Launcher(){

    
    leftShooterMotor = new CANSparkMax(61, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(62, MotorType.kBrushless);

    shooterMotorController = new PIDController(0, 0, 0); //TUNE IT

    shooterFF = new SimpleMotorFeedforward(0,0,0); //TUNEEEEEEE

    configMotors(); 

}


public void shootAmp(){
    leftShooterMotor.setVoltage(-2);
    rightShooterMotor.setVoltage(-2);
}

public void shootSpeaker(){
    leftShooterMotor.setVoltage(-8);
    rightShooterMotor.setVoltage(-8);
    
}

public void launcherStop() {
    leftShooterMotor.setVoltage(0);
    rightShooterMotor.setVoltage(0);
}

public double getLeftVelocity() {
    return leftShooterMotor.getEncoder().getVelocity();
}

public double getRightVelocity() {
    return rightShooterMotor.getEncoder().getVelocity();
}

public void setLauncherVelocity(double setpoint){
    double feedBackLeft = shooterMotorController.calculate(getLeftVelocity(), setpoint);
    double feedBackRight = shooterMotorController.calculate(getRightVelocity(), setpoint);
    double feedForward = shooterFF.calculate(setpoint);
    leftShooterMotor.setVoltage(feedBackLeft + feedForward);
    rightShooterMotor.setVoltage(feedBackRight + feedForward);
}

public double getLeftVoltage(){
    return (leftShooterMotor.getBusVoltage()) * (leftShooterMotor.getAppliedOutput());
}

public double getRightVoltage(){
    return (rightShooterMotor.getBusVoltage()) * (rightShooterMotor.getAppliedOutput());
}

public void configMotors(){
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterMotor.clearFaults();
    rightShooterMotor.clearFaults();

    leftShooterMotor.setSmartCurrentLimit(40);
    rightShooterMotor.setSmartCurrentLimit(40);

    leftShooterMotor.setIdleMode(IdleMode.kBrake);
    rightShooterMotor.setIdleMode(IdleMode.kBrake);

    rightShooterMotor.setInverted(false);
    leftShooterMotor.setInverted(false);

    leftShooterMotor.burnFlash();
    rightShooterMotor.burnFlash();
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
