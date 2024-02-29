package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.proto.Kinematics.ProtobufSwerveModulePosition;
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

    private final JoystickButton driveRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driveLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


    private final JoystickButton driveAprilTagAlignment = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    
    private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton opIntake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final AxisButton opOuttake = new AxisButton(operator, XboxController.Axis.kLeftTrigger.value, 0.5);

    //private final JoystickButton opAmpLauncher = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton opIntakeOverride = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final AxisButton opLauncher = new AxisButton(operator, XboxController.Axis.kRightTrigger.value, 0.5);


    private final POVButton DownDPad = new POVButton(operator, 180);
    // private final POVButton UpDPad = new POVButton(operator, 0);
    // private final POVButton RightDPad = new POVButton(operator, 90);
    // private final POVButton LeftDPad = new POVButton(operator, 270);

    // private final JoystickButton atariButton1 = new JoystickButton(atari, 1);
    // private final JoystickButton atariButton2 = new JoystickButton(atari, 2);
    // private final JoystickButton atariButton3 = new JoystickButton(atari, 3);

    // private final JoystickButton atariButton4 = new JoystickButton(atari, 4);
    // private final JoystickButton atariButton5 = new JoystickButton(atari, 5);
    // private final JoystickButton atariButton6 = new JoystickButton(atari, 6);

    // private final JoystickButton atariButton7 = new JoystickButton(atari, 7);
    // private final JoystickButton atariButton8 = new JoystickButton(atari, 8);

    // private final JoystickButton atariButton11 = new JoystickButton(atari, 11);
    // private final JoystickButton atariButton12 = new JoystickButton(atari, 12);
    //private final JoystickButton OP = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Collector collector = new Collector();
    private final PIDPivot PIDPivot = new PIDPivot();
    private final Launcher launcher = new Launcher();
    private final TurnAndX xLock = new TurnAndX(s_Swerve);

   
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
         //  );


        NamedCommands.registerCommand("Pivot Subwoofer", new InstantCommand(() -> PIDPivot.setSetpointDegrees(9), PIDPivot));
        NamedCommands.registerCommand("Pivot Stage", new InstantCommand(() -> PIDPivot.setSetpointDegrees(28.4), PIDPivot));
        NamedCommands.registerCommand("Pivot Collect", new InstantCommand(() -> PIDPivot.setSetpointDegrees(0), PIDPivot));
        NamedCommands.registerCommand("Pivot Mid Range", new InstantCommand(() -> PIDPivot.setSetpointDegrees(32), PIDPivot));

        NamedCommands.registerCommand("Rev Motors", new InstantCommand(() -> launcher.shootSpeaker(), launcher));
        NamedCommands.registerCommand("Stop Launcher", new InstantCommand(() -> launcher.launcherStop(), launcher));

        NamedCommands.registerCommand("Shoot Note", new InstantCommand(() -> collector.intakeOverride(), collector));
        NamedCommands.registerCommand("Stop Collector", new InstantCommand(() -> collector.collectorStop(), collector));
        
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
        
        // driveA.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        // driveX.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));

        // driveB.whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));
        // driveY.whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));

        opLauncher.onTrue(new InstantCommand(() -> launcher.shootSpeaker(), launcher));
        opLauncher.onFalse(new InstantCommand(() -> launcher.launcherStop(), launcher));
        //opAmpLauncher.onTrue(new InstantCommand(() -> launcher.shootAmp(), launcher));
        //opAmpLauncher.onFalse(new InstantCommand(() -> launcher.launcherStop(), launcher));
        opIntakeOverride.onTrue(new InstantCommand(() -> collector.intakeOverride(), collector));
        opIntakeOverride.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
        
        // opY.onTrue(new InstantCommand(() -> pivot.pivotHold(), PIDPivot));
        // opY.onFalse(new InstantCommand(() -> pivot.pivotStop(), pivot));

        opIntake.onTrue(new CollectNote(collector));
        opIntake.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));
        opOuttake.onTrue(new InstantCommand(() -> collector.collectorOuttake(), collector));
        opOuttake.onFalse(new InstantCommand(() -> collector.collectorStop(), collector));

        // opA.whileTrue(launcher.sysIdQuasistatic(Direction.kForward));
        // opX.whileTrue(launcher.sysIdQuasistatic(Direction.kReverse));

        // opB.whileTrue(launcher.sysIdDynamic(Direction.kForward));
        // opY.whileTrue(launcher.sysIdDynamic(Direction.kReverse));

        // opA.onTrue(new InstantCommand(() -> 
        // pivot.movePivotVolts(SmartDashboard.getNumber("pivotVoltsTest", 0)), pivot));
        // opA.onFalse(new InstantCommand(() -> pivot.pivotHold(), pivot));
       

        opA.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(9), PIDPivot));
        opB.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(32), PIDPivot));
        //opX.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(45), PIDPivot));
        opY.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(85), PIDPivot));
        opIntake.onTrue(new InstantCommand(() -> PIDPivot.setSetpointDegrees(0), PIDPivot));

        DownDPad.whileTrue(new TeleopPivot(PIDPivot, () -> -operator.getRawAxis(translationAxis)));

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
        //return new PathPlannerAuto("Middle3NoteAuto");
    }
}
