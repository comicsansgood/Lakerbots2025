package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
public SparkMax manipulatorMotorTop, manipulatorMotorBottom;
  public ManipulatorSubsystem() {
    manipulatorMotorTop = new SparkMax(2, MotorType.kBrushless);
    manipulatorMotorBottom = new SparkMax(3, MotorType.kBrushless);
  }


  public Command manipulatorGo(double speed) {

    return runOnce(
        () -> {
          manipulatorMotorTop.set(speed);
          manipulatorMotorBottom.set(speed);
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
