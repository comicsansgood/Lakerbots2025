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
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public SparkFlex climberMotor;
  public SparkClosedLoopController closedLoopControllerLeft;
  public SparkMaxConfig motorConfig;
  public double targetPos;



  public Climber() {

    climberMotor = new SparkFlex(5, MotorType.kBrushless);
    

    closedLoopControllerLeft = climberMotor.getClosedLoopController();
  

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);




    motorConfig.closedLoop
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

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

   
    climberMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
   


  }

  public Command climberDown(double targetPos) {
    
    return runOnce(
        () -> {
          climberGoToPosition(Constants.ClimberConstants.climberDown);

        });
  }

  public Command climberUp(double targetPos) {
    
    return runOnce(
        () -> {
          climberGoToPosition(Constants.ClimberConstants.climberUp);
        });
  }

  public Command climberStop(double targetPos) {
    
    return runOnce(
        () -> {
          climberMotor.set(0);
        });
  }

  public Command climberGoToPosition(double targetPos) {
    
    return runOnce(
        () -> {
          this.targetPos = targetPos;
          closedLoopControllerLeft.setReference(targetPos, ControlType.kMAXMotionPositionControl);

        });
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
