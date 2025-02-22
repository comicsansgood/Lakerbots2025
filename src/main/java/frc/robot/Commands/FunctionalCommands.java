package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Manipulator;

public class FunctionalCommands {
    private Manipulator manipulator = new Manipulator();

    public FunctionalCommands(){
        
    }

    public Command doNothing(){
    return Commands.runOnce(()->{});
}

    public FunctionalCommand manipulatorGoToPositionUntilThere(double targetPos){

        return new FunctionalCommand(() -> {
            manipulator.positionController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
        }, ()->{}, interrupt->{}, () -> manipulator.manipulatorAtPosition(), manipulator);
    }
}
