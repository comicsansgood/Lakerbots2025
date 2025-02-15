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

public class Climber2 extends SubsystemBase {

  public SparkMax climber2Motor;
  public SparkClosedLoopController closedLoopController;
  public SparkMaxConfig motorConfig;
  public double targetPos;

  public double tolerance = 0.1;//TODO: tune this value


  public Climber2() {

    climber2Motor = new SparkMax(5, MotorType.kBrushless);

    closedLoopController = climber2Motor.getClosedLoopController();

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);




    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(10.0)
        .i(0)
        .d(1)//TODO:these pid values are trash ngl
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


    //motorConfig.inverted(true);
    climber2Motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  

  public Command climberGoToPosition(double targetPos) {
    
    return runOnce(
        () -> {
          this.targetPos = targetPos;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);

        });
  }

  public boolean climberAtPosition(){
    return getClimberPosition() - targetPos < tolerance;
  }

  public Command climber2Spin(double speed){
    return Commands.runOnce(() -> {climber2Motor.set(speed);});
  }


  public double getClimberPosition(){
    return climber2Motor.getEncoder().getPosition();
  }

  public Command jiggle(double jiggleLength, double timesJiggled){
    return 
    runOnce(()-> {
      for(var i = 0; i < timesJiggled; i++){
        runOnce(() -> {
          targetPos = climber2Motor.getEncoder().getPosition() + jiggleLength;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
        }).until(() -> 
          climberAtPosition()
        ).andThen(() -> {
          targetPos = climber2Motor.getEncoder().getPosition() - jiggleLength;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);  
        });
      }
    });
  }
  @Override
  public void periodic() {
 
  }

  @Override
  public void simulationPeriodic() {
  }
}
