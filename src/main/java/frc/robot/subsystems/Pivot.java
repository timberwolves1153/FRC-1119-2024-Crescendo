// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.units.Angle;
// import edu.wpi.first.units.Distance;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Voltage;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.Constants;
// import edu.wpi.first.units.MutableMeasure;
// import edu.wpi.first.units.Velocity;

// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Volts;
// import static edu.wpi.first.units.Units.RPM;
// import static edu.wpi.first.units.MutableMeasure.mutable;

// public class Pivot extends SubsystemBase{

//     private CANSparkMax m_leftPivotMotor;
//     private CANSparkMax m_rightPivotMotor;
//     //private DigitalInput lowerLimitSwitch;
//     private DutyCycleEncoder pivotAbsoluteEncoder;
//     private SparkPIDController leftPivotMotorPID;
//     private SparkPIDController rightPivotMotorPID;
//     private ArmFeedforward leftPivotFF;
//     private PIDController leftPivotMotor;

//     private SysIdRoutine pivotRoutine = new SysIdRoutine(
//         new SysIdRoutine.Config(), 
//         new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

//     private final MutableMeasure<Voltage> mutableAppliedVoltage = mutable(Volts.of(0));
//     private final MutableMeasure<Angle> mutableDistance = mutable(Radians.of(0));
//     private final MutableMeasure<Velocity<Angle>> mutableVelocity = mutable(RadiansPerSecond.of(0));
    
//     public Pivot() {

//        //super(new PIDController(0.03426, 0, 0));
    
//        m_leftPivotMotor = new CANSparkMax(41, MotorType.kBrushless);
//        m_rightPivotMotor = new CANSparkMax(42, MotorType.kBrushless);

//       // lowerLimitSwitch = new DigitalInput(1);

//        //leftPivotMotor = new PIDController(0.03426, 0, 0); //Tune
//        leftPivotFF = new ArmFeedforward(1.115, 1.87, 1.56, 0.14); //Tune
//        leftPivotMotorPID = m_leftPivotMotor.getPIDController();

//        pivotAbsoluteEncoder = new DutyCycleEncoder(0);

//        configPivot();
//        SmartDashboard.putNumber("pivotVoltsTest", 0);

//     }

//     public void movePivotVolts(double volts) {
//         m_leftPivotMotor.setVoltage(volts);
//     }

//     public void movePivot(double percentPower) {
//         m_leftPivotMotor.setVoltage(2 * percentPower);
//     }
//     public void pivotUp() {
//         m_leftPivotMotor.setVoltage(2);
//     }

//     // public void pivotSetToClimb() {
//     //     m_leftPivotMotor.setVoltage(6);
//     // }

//     public void pivotClimb() {
//         m_leftPivotMotor.setVoltage(-6);
//     }

//     public void pivotDown() {
//         m_leftPivotMotor.setVoltage(-2);
//     }

//     public void pivotStop() {
//         m_leftPivotMotor.setVoltage(0);
//     }

//     public void pivotHold() {
//         double currentPosition = getPivotDegrees();
//         if (currentPosition < 29) { // changed from 34
//             m_leftPivotMotor.setVoltage(.9);
//         } else if (currentPosition < 30) { // changed from 56
//             m_leftPivotMotor.setVoltage(.75);
//         }  else if (currentPosition < 72) {
//             m_leftPivotMotor.setVoltage(.8);
//         }  else if (currentPosition < 91) {
//             m_leftPivotMotor.setVoltage(.7);
//         }  else {
//             m_leftPivotMotor.setVoltage(-.2);
//         }
//     }

   

//     public void teleopPivot(double percentPower) {
//         if (Math.abs(percentPower) < .2) {
//             pivotHold();
//         } else {
//             movePivot(percentPower);
//         }
//     }

//     public double getAbsolutePosition() {
//         return (pivotAbsoluteEncoder.getAbsolutePosition() + .25) % 1;
//     }

//     public double getPivotRadians() {
//         return (getAbsolutePosition() * -2 * Math.PI ) * 26 / 60 + Math.toRadians(143);

//     }

//     public double getPivotDegrees() {
//         return Math.toDegrees(getPivotRadians());
//     }

//     public double getPivotRPM_Degrees(){
//         return m_leftPivotMotor.getEncoder().getVelocity() * 360 / 60;
//     }
    
//     public double getPivotRPM_Radians(){
//         return m_leftPivotMotor.getEncoder().getVelocity() *2 * Math.PI / 60 ;
//     }

//     public void setPivotPosition(double degrees) {
//         double setpointRadians = Math.toRadians(degrees);
//         double feedback = leftPivotMotor.calculate(getPivotRadians(), setpointRadians);
//         double feedforward = leftPivotFF.calculate(setpointRadians, -1);

//         m_leftPivotMotor.setVoltage(feedback + feedforward);
//     }

//     private void voltageDrive(Measure<Voltage> voltage) {
//         m_leftPivotMotor.setVoltage(voltage.in(Volts));
//     }

//     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//             return pivotRoutine.quasistatic(direction);
//         }

//     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//             return pivotRoutine.dynamic(direction);
//         }

//      private void logMotors(SysIdRoutineLog log) {
//         log.motor("pivot")
//             .voltage(mutableAppliedVoltage.mut_replace(
//                 m_leftPivotMotor.getAppliedOutput() * m_leftPivotMotor.getBusVoltage(), Volts))
//             .angularPosition(mutableDistance.mut_replace(getPivotRadians(), Radians))
//             .angularVelocity(mutableVelocity.mut_replace(getPivotRPM_Radians(), RadiansPerSecond));
//     }

    

//     @Override
//     public void periodic() {

//         if (Constants.tunePivot) {
//         SmartDashboard.putNumber("Pivot Degrees", getPivotDegrees());
//         SmartDashboard.putNumber("Pivot Radians", getPivotRadians());
//         SmartDashboard.putNumber("Pivot Absolute Position", getAbsolutePosition());
//         //SmartDashboard.putNumber("Pivot Setpoint", leftPivotMotor.getSetpoint());
//         }

//     }

//     public void configPivot() {
//         m_leftPivotMotor.restoreFactoryDefaults();
//         m_rightPivotMotor.restoreFactoryDefaults();

//         m_leftPivotMotor.clearFaults();
//         m_rightPivotMotor.clearFaults();

//         m_leftPivotMotor.setSmartCurrentLimit(40);
//         m_rightPivotMotor.setSmartCurrentLimit(40);

//         m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
//         m_rightPivotMotor.setIdleMode(IdleMode.kBrake);

//         m_rightPivotMotor.follow(m_leftPivotMotor, true);
//         m_leftPivotMotor.setInverted(false);

//         m_rightPivotMotor.burnFlash();
//         m_leftPivotMotor.burnFlash();
//     }

// }