package frc.robot.subsystems;


import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;

public class Manipulator extends SubsystemBase {

  public LaserCan lazer;
  public SparkMax manipulator;

  public Manipulator(){
    
    lazer = new LaserCan(7);
    manipulator = new SparkMax(8, MotorType.kBrushless);
    
  }
  public Command spinUntilDetected() {

  return runOnce(
    () -> {
      //manipulator.setVoltage(12);
      manipulator.set(0.3);

    });

  /*@SuppressWarnings("static-access")
  public Command spinUntilDetected() {

    return runOnce(
        () -> {
          LaserCan.Measurement measurement = lazer.getMeasurement();
          if (measurement.status == lazer.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 100){
            manipulator.setVoltage(0);
          }
          else{
            manipulator.setVoltage(12);
          }

        }); */
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