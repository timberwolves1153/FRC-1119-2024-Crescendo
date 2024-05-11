package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class DemoSwerve extends Command { 
    
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private Swerve s_Swerve;    
    private Joystick driver;
    private Joystick overide;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSupplier;

    private double speedDampener = 0.25;

    private JoystickButton stopButton;
    private JoystickButton orientationButton;


    public DemoSwerve(Swerve s_Swerve, Joystick driver, Joystick overide) {
        this.s_Swerve = s_Swerve;
        this.driver = driver;
        this.overide = overide;
        addRequirements(s_Swerve);

        stopButton = new JoystickButton(this.overide, XboxController.Button.kA.value);
        orientationButton = new JoystickButton(this.overide, XboxController.Button.kLeftBumper.value);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldCentricSupplier = fieldCentricSupplier;

    }

    @Override
    public void execute() {
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

        /* Get Values, Deadband*/
        if (!stopButton.getAsBoolean()) { 
            translationVal = speedDampener * -Math.pow(MathUtil.applyDeadband(driver.getRawAxis(translationAxis), Constants.stickDeadband), 3);
            strafeVal = speedDampener * -Math.pow(MathUtil.applyDeadband(driver.getRawAxis(strafeAxis), Constants.stickDeadband), 3);
            rotationVal = speedDampener * -Math.pow(MathUtil.applyDeadband(driver.getRawAxis(rotationAxis), Constants.stickDeadband), 3);
        }
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !orientationButton.getAsBoolean(), 
            true
        );
    }
}