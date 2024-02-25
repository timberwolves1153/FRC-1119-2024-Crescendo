package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PIDPivot;

public class TeleopPivot extends Command {
    
    private PIDPivot pivot;
    private DoubleSupplier translationSupplier;

    public TeleopPivot(PIDPivot pivot, DoubleSupplier translationSupplier) {
        this.pivot = pivot;
        addRequirements(pivot);  

        this.translationSupplier = translationSupplier;
    }

     @Override
     public void execute() {
        pivot.teleopPivot(translationSupplier.getAsDouble());
     }
}
