package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CollectNote;
import frc.robot.commands.TeleopPivot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnAndX;
import frc.robot.lib.util.AxisButton;
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
    private final Joystick operator = new Joystick(1);
   // private final Joystick atari = new Joystick(1); 
    private final SendableChooser<Command> autoChooser;
  

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton fieldCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driveClimberUp = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driveClimberDown = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


    private final JoystickButton driveAprilTagAlignment = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    
    private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton opIntake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final AxisButton opOuttake = new AxisButton(operator, XboxController.Axis.kLeftTrigger.value, 0.5);

    private final JoystickButton opIntakeOverride = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final AxisButton opLauncher = new AxisButton(operator, XboxController.Axis.kRightTrigger.value, 0.5);

    private final JoystickButton opSpeakerDistance = new JoystickButton(operator, XboxController.Button.kRightStick.value);


    private final POVButton opTeleopPivot = new POVButton(operator, 180);
    // private final POVButton RightDPad = new POVButton(operator, 90);
    // private final POVButton LeftDPad = new POVButton(operator, 270);
    // private final POVButton UpDPad = new POVButton(operator, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Collector collector = new Collector();
    private final PIDPivot PIDPivot = new PIDPivot();
    private final Launcher launcher = new Launcher();
    private final TurnAndX xLock = new TurnAndX(s_Swerve);
    private final Actuator actuator = new Actuator();

   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
         s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> fieldCentric.getAsBoolean()
            ));

        //  PIDPivot.setDefaultCommand(
        //     new TeleopPivot(
        //         PIDPivot, 
        //         () -> -operator.getRawAxis(translationAxis)
        //     )
        //    );

        NamedCommands.registerCommand("Pivot Subwoofer", new InstantCommand(() -> PIDPivot.setSetpointDegrees(11.25), PIDPivot));
        NamedCommands.registerCommand("Pivot Stage", new InstantCommand(() -> PIDPivot.setSetpointDegrees(28.4), PIDPivot));
        NamedCommands.registerCommand("Pivot Collect", new InstantCommand(() -> PIDPivot.setSetpointDegrees(-3.8), PIDPivot));
        NamedCommands.registerCommand("Pivot Amp", new InstantCommand(() -> PIDPivot.setSetpointDegrees(87)));
        NamedCommands.registerCommand("Pivot Mid Range", new InstantCommand(() -> PIDPivot.setSetpointDegrees(25.65), PIDPivot));
        NamedCommands.registerCommand("Pivot Long Range", new InstantCommand(() -> PIDPivot.setSetpointDegrees(35.2))); //TUNE FIRST
        NamedCommands.registerCommand("Pivot Wing Line", new InstantCommand(() -> PIDPivot.setSetpointDegrees(16.9)));

        NamedCommands.registerCommand("Rev Motors", new InstantCommand(() -> launcher.shootSpeaker(), launcher));   
        NamedCommands.registerCommand("Stop Launcher", new InstantCommand(() -> launcher.launcherStop(), launcher));
        NamedCommands.registerCommand("Shoot Speaker Distance", new InstantCommand(() -> launcher.shootSpeakerDistance(), launcher));

        NamedCommands.registerCommand("Shoot Note", new InstantCommand(() -> collector.intakeOverride(), collector));
        NamedCommands.registerCommand("Stop Collector", new InstantCommand(() -> collector.collectorStop(), collector));
        NamedCommands.registerCommand("Collector Outtake", new InstantCommand(() -> collector.collectorOuttake(), collector));
        
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

        opA.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(10.5), PIDPivot));
        opB.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(32), PIDPivot));
        //opX.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(45), PIDPivot));
        opY.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(86), PIDPivot));
        opIntake.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(-3.8), PIDPivot));

        opTeleopPivot.whileTrue(new TeleopPivot(PIDPivot, () -> -operator.getRawAxis(translationAxis)));

        opLauncher.onTrue(new InstantCommand(() -> launcher.shootSpeaker(), launcher));
        opLauncher.onFalse(new InstantCommand(() -> launcher.launcherStop(), launcher));

        opIntakeOverride.onTrue(new InstantCommand(() -> collector.intakeOverride(), collector));
        opIntakeOverride.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));

        opIntake.onTrue(new CollectNote(collector));
        opIntake.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(-3), PIDPivot));
        opIntake.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
        opIntake.onFalse(new InstantCommand(() -> PIDPivot.setSetpointDegrees(10.5), PIDPivot));

        opOuttake.onTrue(new InstantCommand(() -> collector.collectorOuttake(), collector));
        opOuttake.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));

        opSpeakerDistance.onTrue(new InstantCommand(() -> launcher.shootSpeakerDistance(), launcher));
        opSpeakerDistance.onFalse(new InstantCommand(() -> launcher.launcherStop(), launcher));

        driveClimberUp.onTrue(new InstantCommand(() -> actuator.climberUp(), actuator));
        driveClimberUp.onFalse(new InstantCommand(() -> actuator.climberStop(), actuator));

        driveClimberDown.onTrue(new InstantCommand(() -> actuator.climberDown(), actuator));
        driveClimberDown.onFalse(new InstantCommand(() -> actuator.climberStop(), actuator));

        opX.onTrue(new InstantCommand(() -> 
            PIDPivot.setSetpointDegrees(SmartDashboard.getNumber("Command Setpoint Degrees", 0)),
            PIDPivot));
    }

    public Joystick getDriveController(){
        return driver;
      }

    public Joystick getOperatorController(){
        return operator;
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
        //return new PathPlannerAuto("MiddleCollectAndShootCloseNotes");
    //    return AutoBuilder.followPath(path);
    }
}
