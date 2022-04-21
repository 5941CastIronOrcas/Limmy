package frc.robot;


import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.vision.VisionThread;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
 
public class CodeV6 extends TimedRobot {
   
    public XboxController Controller = new XboxController(0);
    public VictorSP FrontRightMotor = new VictorSP(0);
    public VictorSP RearRightMotor = new VictorSP(1);
    public VictorSP FrontLeftMotor = new VictorSP(2);
    public VictorSP RearLeftMotor = new VictorSP(3);
    public VictorSP ClimberMotor1 = new VictorSP(4);
    public VictorSP ClimberMotor2 = new VictorSP(5);
    public VictorSP LoaderMotor = new VictorSP(6);
    public VictorSP LaunchMotor = new VictorSP(7);
    public VictorSP ArmMotor = new VictorSP(8);
    //public CANSparkMax LoaderMotorCAN = new CANSparkMax(0, MotorType.kBrushless);
    //public CANSparkMax LaunchMotorCAN = new CANSparkMax(1, MotorType.kBrushless);
    //public CANSparkMax ArmMotorCAN = new CANSparkMax(1, MotorType.kBrushless);
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
    public double AutoStuffMultiplier;
    public float LoadTimer;
    public float PreventFiringTimer;
    

    private final Object CAMERA_LOCK = new Object();
    private double cameraContourX;
    
    /*
    Instructions:
    Left stick to move and turn
    Right Bumper: Raise Climber
    Left Bumper: Lower Climber
    Right Trigger (Hold): Enable automatic aiming and ranging
    A Button: Launch Motor
    B Button: Arm Motor
    X Button: Loader Motor
    

    During autonomous, it will Drive backwards for 2 seconds at half voltage, then enable targeting systems.  
    
    Notes: 
    On some controllers (like ours), up on the joystick is negative
    On motors Positive *appears* to be clockwise (but double-check anyway)
    */
   
    public void robotInit() //Does all this when the robot is started
    {
        //Prep Veriables:
        FrontRightMotor.setInverted(true);
        RearRightMotor.setInverted(true);
        //FrontLeftMotor.setInverted(true);
        //RearLeftMotor.setInverted(true);
        LoaderMotor.setInverted(true);
        LaunchMotor.setInverted(true);
        IdealRange = 40;
        TurnMargin = 0.1;
        RangeP = 0.02;
        RangeD = 0.0;
        LockTurnP = 1.0 / 480.0;
        LockTurnD = 0.0005;
        CameraScreenWidth = 640;
        CameraScreenHeight = 480;

        
        
        
        //Create a new USB camera
        UsbCamera camera = CameraServer.startAutomaticCapture();

        //Activate the GripePipeline (Witchcraft)
        new VisionThread(camera, new GripPipeline(), (pipeline) -> {
            MatOfPoint[] objs = pipeline.filterContoursOutput().toArray(new MatOfPoint[0]);
            MatOfPoint largestContour = null;
            double largestContourSize = -1;
            for(int i = 0; i < objs.length; i++) {
                double currentContourSize = objs[i].size().area();
                if(currentContourSize > largestContourSize)
                {
                    currentContourSize = largestContourSize;
                    largestContour = objs[i];
                }
            }
            int camWidth = -1;
            if(largestContour != null)
            {
                Rect rectangle = Imgproc.boundingRect(largestContour);
                camWidth = rectangle.x + (rectangle.width/2);
            }
            //Send some data to the SmartDashboard to look at
            SmartDashboard.putNumber("Target X Position", camWidth);
            SmartDashboard.putNumber("RNG", Math.random());
            SmartDashboard.putNumber("Number Of Targets", objs.length);
            //Witchcraft
            synchronized (CAMERA_LOCK) {
                cameraContourX = camWidth;
            }
        }).start();

        
    }
    public void robotPeriodic() //Does this every 0.02 seconds whenever the robot is running
    {
        //Steal information from Withcraft
        synchronized (CAMERA_LOCK) {
            TargetScreenX = cameraContourX;
        }
        if(TargetScreenX < 0)
        {
            TargetScreenX = CameraScreenWidth / 2.0f;
        }
        //Witchcraft transliterated from Python to make distance sensor work
        int Counter = DistanceSensor.getBytesReceived();
        if(Counter > 8)
        {
          byte[] bytes_serial = DistanceSensor.read(9);
          DistanceSensor.reset();
          if(bytes_serial[0] == 0x59 && bytes_serial[1] == 0x59)
          {
            SensorDistance = bytes_serial[2] + bytes_serial[3] * 256.0;
          }
        }
        else //Insurance against Errors:
        {
            SensorDistance = IdealRange;
            System.out.println("LIDAR is very confused; AutoRanger disabled.");
        }
        if(SensorDistance <= 0)
        {
            SensorDistance = IdealRange;
            System.out.println("Sensor Distance is Erroring to Zero or Negative Value; AutoRanger disabled.");
        }
        SmartDashboard.putNumber("Sensor Distance", SensorDistance);
    }
    
   
    public void teleopPeriodic() //Does this every 0.02 seconds whenever the robot is teleoperated
    {
        LoadTimer -= 0.02f;
        //Use the right trigger to enable locking
        if(Math.abs(Controller.getRightTriggerAxis()) > 0.05)
        {
            LockingEnabled = true;
            AutoStuffMultiplier = Math.abs(Controller.getRightTriggerAxis());
        }
        else
        {
            LockingEnabled = false;
            AutoStuffMultiplier = 0;
        }
        
        //If the driver has locking enabled
        if(LockingEnabled)
        {
           
            //Turning equation for locking (a PID controller):
            LockBasedTurn =  (LockTurnP * (TargetScreenX - (0.5 *  CameraScreenWidth))) + (LockTurnD * ((TargetScreenX - TargetScreenXOld) / 0.02));
           TargetScreenXOld = TargetScreenX;
            //Prevent the AutoAim from becoming too powerful
            if(LockBasedTurn > 1)
            {
                LockBasedMove = 1;
            }
            else if(LockBasedTurn < -1)
            {
                LockBasedMove = -1;
            }
           
           //If in range and on target rumble the controller to tell the driver to shoot
            if(Math.abs(LockBasedTurn) <= TurnMargin && Math.abs(LockBasedMove) <= 0.1)
            {
                Controller.setRumble(RumbleType.kLeftRumble, 1);
                Controller.setRumble(RumbleType.kRightRumble, 1);
            }
            else
            {
                Controller.setRumble(RumbleType.kLeftRumble, 0);
                Controller.setRumble(RumbleType.kRightRumble, 0);
            }
           
            //if pointing close enough to the target, drive forward or backwards to get in the correct range
            if(LockBasedTurn <= TurnMargin)
            {
                LockBasedMove = (-(RangeP * (IdealRange - SensorDistance)) + (RangeD * ((SensorDistance - SensorDistanceOld) / 0.02)));
                SensorDistanceOld = SensorDistance;
            }
            else
            {
                LockBasedMove = 0;
            }
            //Prevent the AutoRanger from becoming too powerful
            if(LockBasedMove > 1)
            {
                LockBasedMove = 1;
            }
            else if(LockBasedMove < -1)
            {
                LockBasedMove = -1;
            }
           
        }
        else
        {
            //If Locking is disabled, do nothing
            LockBasedTurn = 0;
            LockBasedMove = 0;
        }
        //If something has gone horribly wrong, just disable the AutoRanger and/or AutoAimer
        if(Double.isNaN(LockBasedMove))
        {
            LockBasedMove = 0;
        }
        if(Double.isNaN(LockBasedTurn))
        {
            LockBasedTurn = 0;
        }
        //Final Drive motors voltage setting:
        FrontRightMotor.set(-Math.sin(Math.PI * 0.5 * Controller.getLeftY()) - Math.sin(Math.PI * 0.5 * Controller.getLeftX()) + (AutoStuffMultiplier * (-LockBasedTurn + LockBasedMove)));
        RearRightMotor.set(-Math.sin(Math.PI * 0.5 * Controller.getLeftY()) - Math.sin(Math.PI * 0.5 * Controller.getLeftX()) + (AutoStuffMultiplier * (-LockBasedTurn + LockBasedMove)));
        FrontLeftMotor.set(-Math.sin(Math.PI * 0.5 * Controller.getLeftY()) + Math.sin(Math.PI * 0.5 * Controller.getLeftX()) + (AutoStuffMultiplier * (LockBasedTurn + LockBasedMove)));
        RearLeftMotor.set(-Math.sin(Math.PI * 0.5 * Controller.getLeftY()) + Math.sin(Math.PI * 0.5 * Controller.getLeftX()) + (AutoStuffMultiplier * (LockBasedTurn + LockBasedMove)));
 
        //Manual Controls for non-drive motors:
        
        //Climber
        if(Controller.getRightBumper())
        {
            ClimberMotor1.set(1);
            ClimberMotor2.set(1);
        }
        else if(Controller.getLeftBumper())
        {
            ClimberMotor1.set(-1);
            ClimberMotor2.set(-1);
        }
        else
        {
            ClimberMotor1.set(0);
            ClimberMotor2.set(0);
        }
        
        //Launch Sequence
        if(Controller.getAButtonPressed())
        {
            LoadTimer = 1f;
            PreventFiringTimer = 0.25f;
        }
        if(Controller.getAButton() && PreventFiringTimer > 0)
        {
            LoaderMotor.set(-0.25);
        }
        else
        {
            LoaderMotor.set(0);
        }
        if(Controller.getAButton() && PreventFiringTimer < 0)
        {
            LaunchMotor.set(1);
        }
        if(Controller.getAButton() && LoadTimer < 0)
        {
            LoaderMotor.set(1);
        }
        else
        {
            LoaderMotor.set(0);
        }
        if(Controller.getAButtonReleased())
        {
            LaunchMotor.set(0);
            LoaderMotor.set(0);
        }
        
        //Manual Loader control
        if(Controller.getBButtonPressed())
        {
            LoaderMotor.set(1f);
        }
        if(Controller.getBButtonReleased())
        {
            LoaderMotor.set(0f);
        }
        
        //Arm
        if(Controller.getBButton())
        {
            ArmMotor.set(1.0);
        }
        else
        {
            ArmMotor.set(0);
        }

        
        
        
        DebugPort.writeString("Distance: "+SensorDistance +"   "); //Send the distance in centimeters to the debug port
    }
 
    public void autonomousInit() //Does this when autonomous is started
    {
        //Make a variable to see the time since autonomous started
        TimeSinceStartAtAutoStart = Timer.getFPGATimestamp();
        LockBasedMove = 0;
        LockBasedTurn = 0;
    }
    public void autonomousPeriodic() ////Does this every 0.02 seconds whenever the robot is autonomous
    {
        //Drive backwards for 2 seconds, then let loose the targeting systems.  
        if(Timer.getFPGATimestamp() - TimeSinceStartAtAutoStart < 2)
        {
            FrontRightMotor.set(-0.5);
            RearRightMotor.set(-0.5);
            FrontLeftMotor.set(-0.5);
            RearLeftMotor.set(-0.5);
        }
        else
        {
            //Turning equation for targeting the hub (a PID controller):
            LockBasedTurn =  (LockTurnP * (TargetScreenX - (0.5 *  CameraScreenWidth))) + (LockTurnD * ((TargetScreenX - TargetScreenXOld) / 0.02));
           TargetScreenXOld = TargetScreenX;
            //Prevent the AutoAim from becoming too powerful
            if(LockBasedTurn > 1)
            {
                LockBasedTurn = 1;
            }
            else if(LockBasedTurn < -1)
            {
                LockBasedTurn = -1;
            }
           
           //If in range and on target, do something, otherwise do something else
            if(Math.abs(LockBasedTurn) <= TurnMargin && Math.abs(LockBasedMove) <= 0.1)
            {
                
            }
            else
            {
                
            }
           
            //if pointing close enough to the target, drive forward or backwards to get in the correct range
            if(LockBasedTurn <= TurnMargin)
            {
                LockBasedMove = (-(RangeP * (IdealRange - SensorDistance)) + (RangeD * ((SensorDistance - SensorDistanceOld) / 0.02)));
                SensorDistanceOld = SensorDistance;
            }
            else
            {
                LockBasedMove = 0;
            }
            //Prevent the AutoRanger from becoming too powerful
            if(LockBasedMove > 1)
            {
                LockBasedMove = 1;
            }
            else if(LockBasedMove < -1)
            {
                LockBasedMove = -1;
            }

            //Final Drive Motor Voltage Setting:
            FrontRightMotor.set(-LockBasedTurn + LockBasedMove);
            RearRightMotor.set(-LockBasedTurn + LockBasedMove);
            FrontLeftMotor.set(LockBasedTurn + LockBasedMove);
            RearLeftMotor.set(LockBasedTurn + LockBasedMove);
        }
    }
}
 
 
 
