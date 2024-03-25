package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.HashMap;
import java.util.Optional;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PIDPivot extends PIDSubsystem {

    private CANSparkMax m_leftPivotMotor, m_rightPivotMotor;
    private DutyCycleEncoder pivotAbsoluteEncoder;
    private final double unitCircleOffset = 0; // Need to tune this for the pivot radians. ASK how to get the exact value, and if it changes over time
    private Pigeon2 encoder;
    private final double NS_ENCODER_OFFSET = .48;
    private final DigitalInput magnetSwitch;

    private HashMap<Double, Double> degreesToEncoderMap = new HashMap<>();

    public final double COLLECT_SETPOINT = -3;
    public final double TELE_SUBWOOFER_SETPOINT = 10.9;
    public final double AUTO_SUBWOOFER_SETPOINT = 11;
    public final double AMP_SETPOINT = 86;
    public final double AUTO_SHOT_SETPOINT = 1;
    

    public PIDPivot() {
        super(new PIDController(32 , 0, 1));

        m_leftPivotMotor = new CANSparkMax(41, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax(42, MotorType.kBrushless);

        encoder = new Pigeon2(6);
        encoder.clearStickyFaults();

        pivotAbsoluteEncoder = new DutyCycleEncoder(0);
        magnetSwitch = new DigitalInput(3);
        getController().enableContinuousInput(unitCircleOffset, unitCircleOffset);
        configPivot();
        getController().setSetpoint(getMeasurement());
        disable();
 
        SmartDashboard.putNumber("Command Setpoint Degrees", 0);
        SmartDashboard.putNumber("Pivot P", getController().getP());
        SmartDashboard.putNumber("Pivot D", getController().getD());


        //ONLY GAFFEY CAN EDIT THESE VALUES
        SmartDashboard.putNumber("Encoder Offset", NS_ENCODER_OFFSET);
        degreesToEncoderMap.put(COLLECT_SETPOINT,0.122);
        degreesToEncoderMap.put(TELE_SUBWOOFER_SETPOINT, 0.228);//0.242);
        degreesToEncoderMap.put(AUTO_SUBWOOFER_SETPOINT, 0.280); 
        degreesToEncoderMap.put(AMP_SETPOINT,0.682);
        degreesToEncoderMap.put(AUTO_SHOT_SETPOINT,0.290); //0.274);

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
        disable();
        SmartDashboard.putNumber("Piv power", percentPower);
        if (isLimitTriggered() && percentPower < 0) {
            m_leftPivotMotor.setVoltage(0);
        } else {
            m_leftPivotMotor.setVoltage(9 * percentPower);
        }
    }

    public void pivotUp() {
        m_leftPivotMotor.setVoltage(6);
    }

    public void pivotDown() {
        m_leftPivotMotor.setVoltage(-2);
    }

    public void pivotStop() {
        m_leftPivotMotor.setVoltage(0);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        PIDMovePivot(output);
    }

    public boolean isLimitTriggered() {
        return !magnetSwitch.get();
    }

    public void PIDMovePivot(double volts) {
        // invert the given volts since postive makes arm go down
        double adjustedVolts = volts;
        SmartDashboard.putNumber("Piv volts", volts);
        if (isLimitTriggered() && adjustedVolts < 0) {
            adjustedVolts = 0;
            disable();
        }
        // We are good + is up, - is down
        double constantVolts;
        double clampedVolts = MathUtil.clamp(adjustedVolts, -6, 9); //change this according to volts given to the collector(current: 2)
        if (clampedVolts > 0) {
            constantVolts = 0.25; // Needs to be tuned 1153(0.15)
            m_leftPivotMotor.setVoltage(clampedVolts + constantVolts);
        } else if (clampedVolts < 0) {
            constantVolts = 0;// Once again MIGHT needs to be tuned 1153(0)
            m_leftPivotMotor.setVoltage(clampedVolts - constantVolts);
        } else {
            m_leftPivotMotor.setVoltage(0);
        }
        SmartDashboard.putNumber("Pivot output", clampedVolts);
    }

    public double getAbsolutePosition() {
        return 1 - ((pivotAbsoluteEncoder.getAbsolutePosition() + 
            SmartDashboard.getNumber("Encoder Offset", NS_ENCODER_OFFSET) ) % 1);
    }

     public double getPivotRadians() {
    //     // the 60/26 comes from the pivot gear ratios.
         return ((getAbsolutePosition() * -2 * Math.PI) * 60 / 26) % (Math.PI * 2);// + Math.toRadians(134.7);// 102 sets base -> -2 to -3
    //  // +116
    //  // +
     }

    public double getPigeonValues() {
       // return 80.55 - encoder.getPitch().getValueAsDouble();
       return -getRoll();
       //return Math.toDegrees(Math.acos(encoder.getGravityVectorZ().getValueAsDouble()));
    }
    
    @Override
    protected double getMeasurement() {
        //return Math.toRadians(getPivotDegrees());
        //     return getPivotRadians();
        return getAbsolutePosition();
    }

    public double getPivotDegrees() {
        return getPigeonValues();
    }

    public double getYaw() {
        return encoder.getYaw().getValueAsDouble();
    }

    public double getRoll() {
        return encoder.getRoll().getValueAsDouble();
    }

    public double getPitch() {
        return encoder.getPitch().getValueAsDouble();
    }

    public void setSetpointDegrees(double degrees) {
        //double newSetpoint = Math.toRadians(degrees);
        //setSetpoint(newSetpoint); 
        Optional<Double> safeSetPoint = Optional.of(degreesToEncoderMap.get(degrees));

        double setPoint = safeSetPoint.orElse(degreesToEncoderMap.get(-3.0));

        setSetpoint(setPoint);
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
        }  else if (currentPosition < 61) {
            m_leftPivotMotor.setVoltage(.3);
        }  else if (currentPosition < 115) {
            m_leftPivotMotor.setVoltage(.1);
        }  else {
            m_leftPivotMotor.setVoltage(-.2);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Pivot Setpoint", getController().getSetpoint());
        SmartDashboard.putNumber("Pivot Setpoint Degrees", Math.toDegrees(getController().getSetpoint()));
        // SmartDashboard.putNumber("Pivot Degrees", getPigeonValues());
        SmartDashboard.putNumber("pigeon Degrees", encoder.getAngle());
        SmartDashboard.putNumber("pigeon pitch", encoder.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("pigeon roll", encoder.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Final Pigeon Pitch", getPigeonValues());
        SmartDashboard.putNumber("pigeon gravity x", encoder.getGravityVectorX().getValueAsDouble());
        SmartDashboard.putNumber("pigeon arcCos", Math.toDegrees(Math.acos(encoder.getGravityVectorZ().getValueAsDouble())));

        SmartDashboard.putNumber("Abs Encoder Value", getAbsolutePosition());
        SmartDashboard.putNumber("Abs pivot degrees", Math.toDegrees(getPivotRadians()));
        SmartDashboard.putBoolean("magSensor", isLimitTriggered());
        

        // if (Constants.tunePivot) {
        //     SmartDashboard.putNumber("Pivot Degrees", getPivotDegrees());
        //     SmartDashboard.putNumber("Pivot Radians", getPivotRadians());
        //     SmartDashboard.putNumber("Pivot Absolute Position", getAbsolutePosition());
        //     SmartDashboard.putNumber("Pivot Setpoint", getController().getSetpoint());
        //     SmartDashboard.putNumber("Pivot Setpoint Degrees", Math.toDegrees(getController().getSetpoint()));
        // }
    }
}
