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
        public static final double hookPrepare = -70; //-6.5
        public static final double hookLatch = -143; //-16
        public static final double flapCollect = -45; //-5
    }

    public class ClimberConstants{
        public static final double climberHome = 0;
        public static final double climberDown = -180;
        public static final double climberMid = -80;
    }

    public class ElevatorConstants{
        public static final double elevatorHome = -0.5;
        public static final double elevatorCoralL1 = -3;
        public static final double elevatorCoralL2 = -5.5;
        public static final double elevatorCoralL3 = -13;//-14
        public static final double elevatorCoralL4 = -24;  //24 to 25.5        to 24.75
        public static final double elevatorProcess = -2;
        public static final double elevatorAlgaeL2 = -8; // -7 to -8
        public static final double elevatorAlgaeL3 = -14.5;// TODO:make this correct at the real field  
    }

    public class ManipulatorConstants{
        public static final double manipulatorHome = 0;
        public static final double manipulatorCollect = 0;
        public static final double manipulatorTravel = 3;
        public static final double manipulatorCoralL1 = 2;
        public static final double manipulatorCoralL2 = 2;
        public static final double manipulatorCoralL3 = 2;
        public static final double manipulatorCoralL4 = 5;
        public static final double manipulatorAlgeaCollect = 14;
        
        public static final double manipulatorAlgaeProcess = 18;

    }

    public class TagConstants{
        //x and area of tag
        public static final Double[] tagTranslation = {0.0,0.0};//TODO: find these values
    }
    
}
