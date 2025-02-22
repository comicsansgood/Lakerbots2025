package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {

    public static boolean isLazerConnected;

    public static final double wheelBase = 22.75;
    public static final double trackWidth = 20.75;
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    
    

    public class FlapHookConstants{
        public static final double hookflapOpen = 0; // all x 9 for new gear ratio
        public static final double hookPrepare = -58.5; //-6.5
        public static final double hookLatch = -156; //-16
        public static final double flapCollect = -45; //-5
    }

    public class ClimberConstants{
        public static final double climberHome = 0;
        public static final double climberDown = -167;
        public static final double climberMid = -40;
    }

    public class ElevatorConstants{
        public static final double elevatorHome = -0.5;
        public static final double elevatorCoralL1 = -3;
        public static final double elevatorCoralL2 = -5.5;
        public static final double elevatorCoralL3 = -13;
        public static final double elevatorCoralL4 = -25;
        public static final double elevatorProcess = -1;
        public static final double elevatorAlgaeL2 = -7;
        public static final double elevatorAlgaeL3 = -16;
    }

    public class ManipulatorConstants{
        public static final double manipulatorHome = 0;
        public static final double manipulatorCollect = 0;
        public static final double manipulatorTravel = 3.5;
        public static final double manipulatorCoralL1 = 2;
        public static final double manipulatorCoralL2 = 2;
        public static final double manipulatorCoralL3 = 2;
        public static final double manipulatorCoralL4 = 5;
        public static final double manipulatorAlgeaCollect = 16;
        public static final double manipulatorAlgaeProcess = 4;
    }
    
}
