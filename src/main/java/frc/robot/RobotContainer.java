package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnAndX;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick atari = new Joystick(1); 
    private final SendableChooser<Command> autoChooser;
  

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton atariButton4 = new JoystickButton(atari, 4);
    private final JoystickButton atariButton5 = new JoystickButton(atari, 5);
    private final JoystickButton atariButton6 = new JoystickButton(atari, 6);

    private final JoystickButton atariButton1 = new JoystickButton(atari, 1);
    private final JoystickButton atariButton2 = new JoystickButton(atari, 2);
    private final JoystickButton atariButton3 = new JoystickButton(atari, 3);

    private final JoystickButton atariButton11 = new JoystickButton(atari, 11);
    private final JoystickButton atariButton12 = new JoystickButton(atari, 12);
    //private final JoystickButton OP = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot pivot = new Pivot();
    // private final Launcher launcher = new Launcher();
    private final TurnAndX xLock = new TurnAndX(s_Swerve);
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
         s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
        //driveY.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        //driveA.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));
        
        //driveY.whileTrue(xLock);

        // atariButton4.onTrue(new InstantCommand(() -> launcher.shootAmp()));
        // atariButton4.onFalse(new InstantCommand(() -> launcher.launcherStop()));

        // atariButton5.onTrue(new InstantCommand(() -> launcher.shootSpeaker()));
      
        // atariButton5.onFalse(new InstantCommand(() -> launcher.launcherStop()));

        driveA.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        driveX.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));

        driveB.whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));
        driveY.whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));


        

        atariButton1.onTrue(new InstantCommand(() -> pivot.setPivotPosition(0))); //SWITCH THIS FOR Collect
        atariButton2.onTrue(new InstantCommand(() -> pivot.setPivotPosition(0))); //SWITCH THIS FOR Speaker
        atariButton3.onTrue(new InstantCommand(() -> pivot.setPivotPosition(0))); //SWITCH THIS FOR AMP

        atariButton11.onTrue(new InstantCommand(() -> pivot.pivotForward()));
        atariButton11.onFalse(new InstantCommand(() -> pivot.pivotStop()));

        atariButton12.onTrue(new InstantCommand(() -> pivot.pivotBackward()));
        atariButton12.onFalse(new InstantCommand(() -> pivot.pivotStop()));

    }

    public Joystick getDriveController(){
        return driver;
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return autoChooser.getSelected();
        return new PathPlannerAuto("StraightLine");
    //    PathPlannerPath path = PathPlannerPath.fromPathFile("StraightLine");

    //    return AutoBuilder.followPath(path);
    }
}