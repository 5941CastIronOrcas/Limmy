package frc.robot;
 
 
 
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
 
public class CodeV5 extends TimedRobot {
   
    public XboxController Controller = new XboxController(0);
    public VictorSP FrontRightMotor = new VictorSP(0);
    public VictorSP RearRightMotor = new VictorSP(1);
    public VictorSP FrontLeftMotor = new VictorSP(2);
    public VictorSP RearLeftMotor = new VictorSP(3);
    public VictorSP LaunchMotor1 = new VictorSP(4);
    public VictorSP LaunchMotor2 = new VictorSP(5);
    public VictorSP IntakeMotorMain = new VictorSP(6);
    public VictorSP IntakeMotorSecondary = new VictorSP(7);
    public VictorSP LoaderMotor = new VictorSP(8);
    public double TargetScreenX;
    public double TargetScreenXOld;
    public double TargetScreenY;
    public boolean LockingEnabled;
    public double LockBasedTurn;
    public double LockBasedMove;
    public double LockTurnP;
    public double LockTurnD;
    public double CameraScreenWidth;
    public double CameraScreenHeight;
    public double TimeSinceStartAtAutoStart;
    public Accelerometer AccelerometerMain = new BuiltInAccelerometer();
    private final SerialPort DistanceSensor = new SerialPort(115200, SerialPort.Port.kMXP);
    private final SerialPort DebugPort = new SerialPort(115200, SerialPort.Port.kOnboard);
    public double SensorDistance;
    public double SensorDistanceOld;
    public double IdealRange;
    public double TurnMargin;
    public double RangeP;
    public double RangeD;
   
    /*
    Instructions:
    Left stick to move and turn
    Hold Right Trigger to enable Locking
    Hold A to Fire
    Left Trigger to use Main loader motor
    Right Trigger to use Secondary loader motor
    
    Notes: 
    On some controllers (like ours), up on the joystick is negative
    On motors Positive *appears* to be clockwise (but double-check anyway)
    */
   
    public void robotInit()
    {
        FrontRightMotor.setInverted(true);
        RearRightMotor.setInverted(true);
        //FrontLeftMotor.setInverted(true);
        //RearLeftMotor.setInverted(true);
        CameraServer.startAutomaticCapture();
        IdealRange = 100;
        TurnMargin = 0.1;
        RangeP = .01;
        RangeD = 0;
        LockTurnP = 0;
        LockTurnD = 0;
        
    }
    public void teleopInit()
    {
   
    }
   
    public void teleopPeriodic()
    {
       
        int Counter = DistanceSensor.getBytesReceived();
        if(Counter > 8)
        {
          byte[] bytes_serial = DistanceSensor.read(9);
          DistanceSensor.reset();
          if(bytes_serial[0] == 0x59 && bytes_serial[1] == 0x59)
          {
            SensorDistance = Math.max(bytes_serial[2] + bytes_serial[3] * 256, 0);
          }
        }
        else
        {
            SensorDistance = IdealRange;
        }

        if(Double.isNaN(LockBasedMove))
        {
            LockBasedMove = 0;
        }
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
           
            //Turning equation for locking:
            LockBasedTurn =  (LockTurnP * (TargetScreenX - (0.5 *  CameraScreenWidth))) + (LockTurnD * ((TargetScreenX - TargetScreenXOld) / 0.02));
           TargetScreenXOld = TargetScreenX;
           
           //If in range and on target:
            if(LockBasedTurn <= TurnMargin && LockBasedMove <= 0.1)
            {
                Controller.setRumble(RumbleType.kLeftRumble, 1);
                Controller.setRumble(RumbleType.kRightRumble, 1);
            }
            else
            {
                Controller.setRumble(RumbleType.kLeftRumble, 0);
                Controller.setRumble(RumbleType.kRightRumble, 0);
            }
           
            //if out of range, get in range
            if(LockBasedTurn <= TurnMargin)
            {
                LockBasedMove = (-(RangeP * (IdealRange - SensorDistance)) + (RangeD * ((SensorDistance - SensorDistanceOld) / 0.02)));
                SensorDistanceOld = SensorDistance;  
            }
            else
            {
                LockBasedMove = 0;
            }
           
        }
        else
        {
            //If Locking is disabled, do nothing
            LockBasedTurn = 0;
            LockBasedMove = 0;
        }
        //Final Drive motors voltage setting:
        FrontRightMotor.set(-Controller.getLeftY() - Controller.getLeftX() - LockBasedTurn + LockBasedMove);
        RearRightMotor.set(-Controller.getLeftY() - Controller.getLeftX() - LockBasedTurn + LockBasedMove);
        FrontLeftMotor.set(-Controller.getLeftY() + Controller.getLeftX() + LockBasedTurn + LockBasedMove);
        RearLeftMotor.set(-Controller.getLeftY() + Controller.getLeftX() + LockBasedTurn + LockBasedMove);
 
        //Manual Controls:
        if(Controller.getAButton())
        {
            LaunchMotor1.set(1);
            LaunchMotor2.set(1);
            LoaderMotor.set(1);
        }
        else
        {
            LaunchMotor1.set(0);
            LaunchMotor2.set(0);
            LoaderMotor.set(0);
        }
        IntakeMotorMain.set(Controller.getLeftTriggerAxis());
        if(Controller.getBButton())
        {
            IntakeMotorSecondary.set(1);
        }
        else
        {
            IntakeMotorSecondary.set(0);
        }
        LaunchMotor1.set(SensorDistance);
        DebugPort.writeString("Distance: "+SensorDistance +"   "); //Send the distance in CM to the debug port
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
 
 
 
