package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlapHook extends SubsystemBase {

  public SparkMax flapHookMotor;
  public SparkClosedLoopController closedLoopController;
  public SparkMaxConfig motorConfig;
  public double targetPos;
  private RelativeEncoder encoder; 

  public double tolerance = 0.1;//TODO: tune this value


  public FlapHook() {

    flapHookMotor = new SparkMax(8, MotorType.kBrushless);

    closedLoopController = flapHookMotor.getClosedLoopController();

    encoder =flapHookMotor.getEncoder(); 

    motorConfig = new SparkMaxConfig();



    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    motorConfig.idleMode(IdleMode.kBrake);

    motorConfig.smartCurrentLimit(40);


    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.13)
        .i(0)
        .d(1)//TODO:these pid values are trash ngl
        .outputRange(-1, 1);
        // Set PID values for velocity control in slot 1
       // .p(0.0001, ClosedLoopSlot.kSlot1)
        //.i(0, ClosedLoopSlot.kSlot1)
        //.d(0, ClosedLoopSlot.kSlot1)
        //.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        //.outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(5000)
        .maxAcceleration(5000)
        .allowedClosedLoopError(0.1);
        // Set MAXMotion parameters for velocity control in slot 1
        //.maxAcceleration(500, ClosedLoopSlot.kSlot1)
        //.maxVelocity(6000, ClosedLoopSlot.kSlot1)
       // .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot1);

    motorConfig.softLimit
    .reverseSoftLimit(-143) // -16 x 9 for gear ratio change
    .reverseSoftLimitEnabled(true)
    .forwardSoftLimit(143)
    .forwardSoftLimitEnabled(true);


    //motorConfig.inverted(true);
    flapHookMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    
    encoder.setPosition(0);





  }

  public Command hookGoIn(double targetSpeed){
    return runOnce(
      () -> {
        flapHookMotor.set(targetSpeed);
      }
    );
  }

  public Command hookGoOut(double targetSpeed){
    return runOnce(
      () -> {
        flapHookMotor.set(targetSpeed);
      }
    );
  }

  public Command hookStop(double targetSpeed){
    return runOnce(
      () -> {
        flapHookMotor.set(targetSpeed);
      }
    );
  }

  public Command hookGoToPosition(double targetPos) {
    
    return runOnce(
        () -> {
          this.targetPos = targetPos;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);

        });
  }

  public boolean hookAtPosition(){
    return Math.abs(getHookPosition() - targetPos) < tolerance;
  }

  public Command flapHookSpin(double speed){
    return Commands.runOnce(() -> {flapHookMotor.set(speed);});
  }


  public double getHookPosition(){
    return flapHookMotor.getEncoder().getPosition();
  }

  public Command flapHookEncoderReset(){
    return Commands.runOnce(() ->{encoder.setPosition(0);});
  }

  public Command jiggle(double jiggleLength, double timesJiggled){
    return 
    runOnce(()-> {
      for(var i = 0; i < timesJiggled; i++){
        runOnce(() -> {
          targetPos = flapHookMotor.getEncoder().getPosition() + jiggleLength;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
        }).until(() -> 
          hookAtPosition()
        ).andThen(() -> {
          targetPos = flapHookMotor.getEncoder().getPosition() - jiggleLength;
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
