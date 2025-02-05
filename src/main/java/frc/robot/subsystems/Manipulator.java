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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;

public class Manipulator extends SubsystemBase {

  public LaserCan lazer;
  public SparkMax manipulatorSpin;
  public SparkMax manipulatorWrist;
  public SparkClosedLoopController positionController;
  public SparkMaxConfig wristConfig;

  public boolean isCoralDetected;

  public Manipulator(){
    
    lazer = new LaserCan(7);
    manipulatorSpin = new SparkMax(8, MotorType.kBrushless);

    manipulatorWrist = new SparkMax(99, MotorType.kBrushless);//TODO:can id
    

    positionController = manipulatorWrist.getClosedLoopController();
  

    wristConfig = new SparkMaxConfig();

    wristConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);




    wristConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    wristConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

   
    manipulatorWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
   

    
  }
  public Command spinUntilDetected() {

  return runEnd(
    () -> {
      manipulatorSpin.set(0.3);
    },
    () -> {
      manipulatorSpin.set(0);
    }).until(() -> isCoralDetected);



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
    isCoralDetected = lazer.getMeasurement().distance_mm <= 100;
    SmartDashboard.putBoolean("isCoral", isCoralDetected);
  }

  @Override
  public void simulationPeriodic() {
  }
}