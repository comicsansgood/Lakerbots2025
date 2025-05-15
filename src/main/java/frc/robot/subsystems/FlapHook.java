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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlapHook extends SubsystemBase {

  //Subsystem objects
  public SparkMax flapHookMotor;
  public SparkClosedLoopController closedLoopController;
  public SparkMaxConfig motorConfig;
  public double targetPos;
  private RelativeEncoder encoder; 

  public double tolerance = 0.1;


  //Bookmark-2a
  public FlapHook() {

    //objects
    flapHookMotor = new SparkMax(8, MotorType.kBrushless);
    closedLoopController = flapHookMotor.getClosedLoopController();
    encoder = flapHookMotor.getEncoder(); 
    //config
    motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    motorConfig.idleMode(IdleMode.kBrake);

    motorConfig.smartCurrentLimit(40);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.13)
        .i(0)
        .d(1)
        .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(5000)
        .maxAcceleration(5000)
        .allowedClosedLoopError(0.1);


    motorConfig.softLimit
    .reverseSoftLimit(-143) //In this case, this should be the same as the flaphook closed postion                    
    .reverseSoftLimitEnabled(true)
    .forwardSoftLimit(143)
    .forwardSoftLimitEnabled(true);

    flapHookMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder.setPosition(0);
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

  //unused but funny so i left it in
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
