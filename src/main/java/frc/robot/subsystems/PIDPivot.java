package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PIDPivot extends PIDSubsystem {

    private CANSparkMax m_leftPivotMotor, m_rightPivotMotor;
    private DutyCycleEncoder pivotAbsoluteEncoder;
    private final double unitCircleOffset = 0; // Need to tune this for the pivot radians. ASK how to get the exact
                                               // value, and if it changes over time/

    public PIDPivot() {
        super(new PIDController(55, 0, 1));//REEEETUNEEEE

        m_leftPivotMotor = new CANSparkMax(41, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax(42, MotorType.kBrushless);

        pivotAbsoluteEncoder = new DutyCycleEncoder(0);
        getController().enableContinuousInput(unitCircleOffset, unitCircleOffset);
        configPivot();
        getController().setSetpoint(getMeasurement());
        disable();

        SmartDashboard.putNumber("Command Setpoint Degrees", 0);
        SmartDashboard.putNumber("Pivot P", getController().getP());
        SmartDashboard.putNumber("Pivot D", getController().getD());

        
    }

    public void teleopPivot(double percentPower) {
        if (Math.abs(percentPower) < .2) {
            pivotHold();
        } else {
            movePivot(percentPower);
        }
    }

    public void configPivot() {
        m_leftPivotMotor.restoreFactoryDefaults();
        m_rightPivotMotor.restoreFactoryDefaults();

        m_leftPivotMotor.clearFaults();
        m_rightPivotMotor.clearFaults();

        m_leftPivotMotor.setSmartCurrentLimit(40);
        m_rightPivotMotor.setSmartCurrentLimit(40);

        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.setIdleMode(IdleMode.kBrake);

        m_rightPivotMotor.follow(m_leftPivotMotor, true);
        m_leftPivotMotor.setInverted(false);

        m_rightPivotMotor.burnFlash();
        m_leftPivotMotor.burnFlash();
    }

    public void movePivotVolts(double volts) {
        m_leftPivotMotor.setVoltage(volts);
    }

    public void movePivot(double percentPower) {
        m_leftPivotMotor.setVoltage(2 * percentPower);
    }

    public void pivotUp() {
        m_leftPivotMotor.setVoltage(2);
    }

    public void pivotDown() {
        m_leftPivotMotor.setVoltage(-2);
    }

    public void pivotStop() {
        m_leftPivotMotor.setVoltage(0);
    }

    public void pivotClimbUp(){
        m_leftPivotMotor.setVoltage(6);
    }

    public void pivotClimbDown(){
        m_leftPivotMotor.setVoltage(-6);
    }



    @Override
    protected void useOutput(double output, double setpoint) {
        PIDMovePivot(output);
    }

    public void PIDMovePivot(double volts) {
        // invert the given volts since postive makes arm go down
        double adjustedVolts = volts;
        // We are good + is up, - is down
        double constantVolts;
        double clampedVolts = MathUtil.clamp(adjustedVolts, -6, 5); //change this according to volts given to the collector(current: 2)
        if (clampedVolts > 0) {
            constantVolts = 0; // Needs to be tuned 1153(0.15)
            m_leftPivotMotor.setVoltage(clampedVolts + constantVolts);
        } else if (clampedVolts < 0) {
            constantVolts = 0;// Once again MIGHT needs to be tuned 1153(0)
            m_leftPivotMotor.setVoltage(clampedVolts - constantVolts);
        }
        SmartDashboard.putNumber("Pivot output",clampedVolts);
    }

    @Override
    protected double getMeasurement() {
        return getPivotRadians();
    }

    public double getAbsolutePosition() {
        return (pivotAbsoluteEncoder.getAbsolutePosition() + .25) % 1;
    }

    public double getPivotRadians() {
        // the 60/26 comes from the pivot gear ratios.
        return (getAbsolutePosition() * -2 * Math.PI) * 26 / 60 + Math.toRadians(97.25);// 102 sets base -> -2 to -3
    }

    public double getPivotDegrees() {
        return Math.toDegrees(getPivotRadians());
    }

    public void setSetpointDegrees(double degrees) {
        double newSetpoint = Math.toRadians(degrees);
        setSetpoint(newSetpoint);        
        getController().reset();
        getController().setP(SmartDashboard.getNumber("Pivot P", getController().getP()));
        getController().setD(SmartDashboard.getNumber("Pivot D", getController().getD()));;
        enable();
    }

        public void pivotHold() {
        double currentPosition = getPivotDegrees();
        if (currentPosition < 29) { // changed from 34
            m_leftPivotMotor.setVoltage(.5);
        } else if (currentPosition < 30) { // changed from 56
            m_leftPivotMotor.setVoltage(.5);
        }  else if (currentPosition <61) {
            m_leftPivotMotor.setVoltage(.3);
        }  else if (currentPosition < 91) {
            m_leftPivotMotor.setVoltage(.1);
        }  else {
            m_leftPivotMotor.setVoltage(-.2);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        if (Constants.tunePivot) {
            SmartDashboard.putNumber("Pivot Degrees", getPivotDegrees());
            SmartDashboard.putNumber("Pivot Radians", getPivotRadians());
            SmartDashboard.putNumber("Pivot Absolute Position", getAbsolutePosition());
            SmartDashboard.putNumber("Pivot Setpoint", getController().getSetpoint());
            SmartDashboard.putNumber("Pivot Setpoint Degrees", Math.toDegrees(getController().getSetpoint()));
        }
    }
}
