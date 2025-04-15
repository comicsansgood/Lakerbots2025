package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;

public class Elevator extends SubsystemBase {

  //Objects
  public TalonFX elevatorLead = new TalonFX(13, "canivore");
  public TalonFX elevatorFollow = new TalonFX(14, "canivore");
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final DynamicMotionMagicVoltage dynamicReq = new DynamicMotionMagicVoltage(0, 80, 400, 4000);

  //Config
  public TalonFXConfiguration cfg = new TalonFXConfiguration();

  private final Mechanisms m_mechanisms = new Mechanisms();

  public double setpoint;
  public double tolerance = 1;


  public Elevator() {
    //Soft limit switch
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -31;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1;

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(15)) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15)) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));


    //PID values
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.275; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 1.0;//0.6 // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0.0;//0.05 // No output for integrated error
    slot0.kD = 0.0;//0.2 // A velocity error of 1 rps results in 0.5 V output
    slot0.kG = 0.5; 

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorLead.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    //command elevatorFollow to follow elevatorLead
    elevatorFollow.setControl(new Follower(elevatorLead.getDeviceID(), false));//change bool for dir inversion   
  }

  public Command elevatorGoToPosition(double setpoint){
    return Commands.runOnce(() -> {
    this.setpoint = setpoint;
    elevatorLead.setControl(m_mmReq.withPosition(setpoint));
    });
  }

  public double elevatorGetError(){
    return elevatorLead.getPosition().getValueAsDouble() - setpoint;
  }


  //For using 2 different v, a, and j values for going up and down
  //Dynamic Motion Magic (Pheonix Pro) is needed for this
  //Two functions, one is used for up and one is used for down

  public void elevatorUpDynamic(double setpoint){
    this.setpoint = setpoint;
    dynamicReq.Velocity = 45;//60
    dynamicReq.Acceleration = 50;//80
    dynamicReq.Jerk = 0;
    //dynamicReq.FeedForward = 100;
    elevatorLead.setControl(dynamicReq.withPosition(setpoint));
  }
  public void elevatorDownDynamic(double setpoint){
    this.setpoint = setpoint;
    dynamicReq.Velocity = 26;
    dynamicReq.Acceleration = 26;
    dynamicReq.Jerk = 0;
    //dynamicReq.FeedForward = 100;
    elevatorLead.setControl(dynamicReq.withPosition(setpoint));
  }

  //non dynamic motion
  public Command elevatorGoToPositionUntilThere(double setpoint){
    return runEnd(
      () -> {this.setpoint = setpoint;elevatorLead.setControl(m_mmReq.withPosition(setpoint));},
      () -> {this.setpoint = setpoint;elevatorLead.setControl(m_mmReq.withPosition(setpoint));}
    ).until(() -> elevatorAtPosition());
  }

  public Command elevatorDownUntilThereDynamic(double setpoint){
    return runEnd(
      () -> {dynamicReq.Velocity = 60;
        dynamicReq.Acceleration = 80;
        dynamicReq.Jerk = 0;
        elevatorLead.setControl(dynamicReq.withPosition(setpoint));},
      () -> {elevatorLead.setControl(dynamicReq.withPosition(setpoint));}
      ).until(() -> elevatorAtPosition());  
    }

  public boolean elevatorAtPosition(){
    return tolerance > Math.abs(elevatorLead.getPosition().getValueAsDouble() - setpoint);
  }

  public double elevatorGetCurrent(){
    return elevatorLead.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("elevatorAtPosition", elevatorAtPosition());
    SmartDashboard.putNumber("elevatorError", elevatorGetError());
    SmartDashboard.putNumber("elevator position", elevatorLead.getPosition().getValueAsDouble());

    m_mechanisms.update(elevatorLead.getPosition(), elevatorLead.getVelocity());
  }

  @Override public void simulationPeriodic() {
  }
}
