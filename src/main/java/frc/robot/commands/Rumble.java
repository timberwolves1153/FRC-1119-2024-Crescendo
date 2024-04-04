package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Rumble extends Command {
    
    private Collector collector;
    private boolean limit;
    private Joystick driver;
    private Joystick operator;

    public Rumble (Collector collector, Joystick driver, Joystick operator){

        this.collector = collector;
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void execute() {
        if (collector.hasNote()) {
            driver.setRumble(RumbleType.kBothRumble, 1);
            operator.setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.setRumble(RumbleType.kBothRumble, 0);
            operator.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);
    }
}