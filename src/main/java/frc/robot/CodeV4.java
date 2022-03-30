package frc.robot;
 
 
 
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
 
public class CodeV4 extends TimedRobot {
   
    public XboxController Controller = new XboxController(0);
    public VictorSP FrontRightMotor = new VictorSP(0);
    public VictorSP RearRightMotor = new VictorSP(1);
    public VictorSP FrontLeftMotor = new VictorSP(2);
    public VictorSP RearLeftMotor = new VictorSP(3);
    public VictorSP LaunchMotor1 = new VictorSP(4);
    public Servo SensorBoomServo = new Servo(5);
    public Servo ArmamentBoomServo = new Servo(6);
    public double TargetScreenX;
    public double TargetScreenY;
    public double TargetRealZ;
    public double TargetRealY;
    public boolean LockingEnabled;
    public double LockBasedTurn;
    public double LockBasedMove;
    public double LockTurnSensitivity;
    public double CameraScreenWidth;
    public double CameraScreenHeight;
    public double LaunchSpeed;
    public double LaunchAngle;
    public double TargetDistanceDirect;
    public double TimeSinceStartAtAutoStart;
    public double ArmamentBoomServoRealAngle;
    public Accelerometer AccelerometerMain = new BuiltInAccelerometer();
   
    /*
    Instructions:
    Left stick to move, right to turn
    Hold the right trigger to attempt to acquire a lock on a target on the screen
    controller vibrates when locked
    hold A to activate launch motor(s)
    In auto, it simply drives backward for 2 seconds, if the targeting code works out then
    I can add it to auto later.  
    Note: While locked/locking, you are still able to drive manually, although this will likely destabilize the lock.  
    */
   
    public void robotInit()
    {
        FrontRightMotor.setInverted(true);
        RearRightMotor.setInverted(true);
        //FrontLeftMotor.setInverted(true);
        //RearLeftMotor.setInverted(true);
        CameraServer.startAutomaticCapture();
    }
    public void teleopInit()
    {
   
    }
   
    public void teleopPeriodic()
    {
   
        if(Math.abs(Controller.getRightTriggerAxis()) > 0.25)
        {
            LockingEnabled = true;
        }
        else
        {
            LockingEnabled = false;
        }
 
        if(LockingEnabled)
        {
            if(TargetScreenY > 0)
            {
                SensorBoomServo.setAngle(SensorBoomServo.getAngle() + 1);
            }
            else if(TargetScreenY < 0)
            {
                SensorBoomServo.setAngle(SensorBoomServo.getAngle() - 1);
            }
           
            TargetRealZ = TargetDistanceDirect * Math.cos(SensorBoomServo.getAngle());
            TargetRealY = TargetDistanceDirect * Math.sin(SensorBoomServo.getAngle());
            //Main angle equation:
            LaunchAngle = Math.atan((Math.pow(LaunchSpeed, 2) + Math.sqrt(Math.pow(LaunchSpeed, 4)
            - 9.81 * (9.81*Math.pow(TargetRealZ, 2) + 2*TargetRealY*Math.pow(LaunchSpeed, 2))))
            / (9.81*TargetRealZ));
            ArmamentBoomServo.setAngle(LaunchAngle);
 
            //Turning equation for locking:
            LockBasedTurn = TargetScreenX - (LockTurnSensitivity * 0.5 *  CameraScreenWidth);
           
           
           //If in range and on target:
            if(!Double.isNaN(LaunchAngle) &&
            Math.abs(TargetScreenX) < 4 &&
            Math.abs(TargetScreenY) < 4 &&
            Math.abs(ArmamentBoomServoRealAngle) - Math.abs(ArmamentBoomServo.getAngle()) < 4)
            {
                Controller.setRumble(RumbleType.kLeftRumble, 1);
                Controller.setRumble(RumbleType.kRightRumble, 1);
            }
            else
            {
                Controller.setRumble(RumbleType.kLeftRumble, 1);
                Controller.setRumble(RumbleType.kRightRumble, 1);
            }
           
            //if out of range, get in range
            if(!Double.isNaN(LaunchAngle))
            {
                LockBasedMove = 0;
            }
            else
            {
                LockBasedMove = 1;
            }
            //Manual activation of launch motor:
            if(Controller.getAButton())
            {
                LaunchMotor1.set(1);
            }
            else
            {
                LaunchMotor1.set(0);
            }
        }
        else
        {
            //If Locking is disabled, do nothing
            LockBasedTurn = 0;
            LockBasedMove = 0;
        }
        //Final Drive motors voltage setting:
        FrontRightMotor.set(Controller.getLeftY() - Controller.getLeftX() - LockBasedTurn + LockBasedMove);
        RearRightMotor.set(Controller.getLeftY() - Controller.getLeftX() - LockBasedTurn + LockBasedMove);
        FrontLeftMotor.set(Controller.getLeftY() + Controller.getLeftX() + LockBasedTurn + LockBasedMove);
        RearLeftMotor.set(Controller.getLeftY() + Controller.getLeftX() + LockBasedTurn + LockBasedMove);
    }
 
    public void autonomousInit()
    {
        TimeSinceStartAtAutoStart = Timer.getFPGATimestamp();
    }
    public void autonomousPeriodic()
    {
        if(Timer.getFPGATimestamp() - TimeSinceStartAtAutoStart < 2)
        {
            FrontRightMotor.set(-0.5);
            RearRightMotor.set(-0.5);
            FrontLeftMotor.set(-0.5);
            RearLeftMotor.set(-0.5);
        }
    }
}
