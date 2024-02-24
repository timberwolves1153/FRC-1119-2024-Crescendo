package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDPivot extends PIDSubsystem{
    
    private CANSparkMax m_leftPivotMotor, m_rightPivotMotor;
    private DutyCycleEncoder pivotAbsoluteEncoder;

    public PIDPivot() {

        super(new PIDController(10, 0.01, 0.01));

       m_leftPivotMotor = new CANSparkMax(41, MotorType.kBrushless);
       m_rightPivotMotor = new CANSparkMax(42, MotorType.kBrushless);

       pivotAbsoluteEncoder = new DutyCycleEncoder(0);
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
}
