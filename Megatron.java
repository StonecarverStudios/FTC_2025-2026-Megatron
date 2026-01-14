import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



@TeleOp(name="Megatron")
public class Megatron extends LinearOpMode {
    
    DcMotorEx m1, m2, m3, m4, leftThrow, rightThrow, topFeeder, bottomFeeder;
    
    Servo trigger;
    
    //DistanceSensor topFeederSensor;
    
    ElapsedTime runTime = new ElapsedTime();
    
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    double lastAutoTurn = 0;

    
    @Override
    public void runOpMode(){
        
        // Wheels, ordered accordingly:
        // Top Left, Back Left, Front Right, Back Right
        m1 = hardwareMap.get(DcMotorEx.class, "bl"); //back left
        m2 = hardwareMap.get(DcMotorEx.class, "br"); //back right
        m3 = hardwareMap.get(DcMotorEx.class, "fr"); //front right
        m4 = hardwareMap.get(DcMotorEx.class, "fl"); //front left
        
        //topFeederSensor = hardwareMap.get(DistanceSensor.class, "BallDistanceSensor");

        
        //Launcher motors
        leftThrow = hardwareMap.get(DcMotorEx.class, "leftThrow");
        rightThrow = hardwareMap.get(DcMotorEx.class, "rightThrow");
    
        //Trigger Servot
        trigger = hardwareMap.get(Servo.class, "trigger");
        
        //Feeder motor
        topFeeder = hardwareMap.get(DcMotorEx.class, "topFeeder");
        bottomFeeder = hardwareMap.get(DcMotorEx.class, "bottomFeeder");
        
        
        //Motor direction, flip to reverse direction of robot (may need to reorder motors)
        //m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        
        leftThrow.setDirection(DcMotor.Direction.REVERSE);
        
        leftThrow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightThrow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftThrow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightThrow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
        
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Reset the encoders on Init, and set them to the correct mode 
        // m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        
        
        //WebCam init
        aprilTagWebcam.init(hardwareMap, telemetry);
        
        
        //init triger
        trigger.setPosition(0.6);
        
        boolean farRange = true; // default to far range throw
        
        telemetry.addData("Press Start When Ready","");
        telemetry.addData("Far Range Mode Activated","");
        telemetry.update();
        
        boolean locked = false;
        
        boolean isLogged = false;
        
        //Power to the throwing motors
        double closePower = 0.02;
        double farPower = 0.73;

       
        boolean yLastPressed = false;
        boolean xLastPressed = false;
      
        waitForStart();
        runTime.reset();

        while (opModeIsActive()) {
            
            
            
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20); //Blue Goal
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24); //Red Goal
            
            double BEARING_OFFSET;
            
            // ================= AIMBOT TURN LOGIC =================
            if (farRange){
                BEARING_OFFSET = 7.0; // degrees (positive / Aim Left)
            }else{
                BEARING_OFFSET = 0;
            }
            
            if (id24 != null) {
                BEARING_OFFSET = 7.0;
            }else if (id20 != null){
                BEARING_OFFSET = 1.5;
            }
            
           
            
            double manualTurn = gamepad1.left_trigger - gamepad1.right_trigger;
            double autoTurn = 0;
            
            double AIM_KP = 0.02;        // proportional gain
            double DEAD_BAND = 1.5;      // degrees
            double MAX_TURN = 0.5;       // max auto turn power
            double MIN_TURN = 0.05;      // minimum motor power
            double SMOOTHING = 0.15;     // 0–1 (higher = snappier)
            
            // Only run auto-aim if Y is pressed
            if (gamepad1.y) {
                Double bearing = null;
            
                // Prioritize red side (id24) if present, otherwise use id20
                if (id24 != null) {
                    bearing = id24.ftcPose.bearing + BEARING_OFFSET;
                } else if (id20 != null) {
                    bearing = id20.ftcPose.bearing + BEARING_OFFSET;
                }
            
                if (bearing != null && Math.abs(bearing) > DEAD_BAND) {
                    autoTurn = bearing * AIM_KP;
            
                    // Clamp
                    autoTurn = Math.max(-MAX_TURN, Math.min(MAX_TURN, autoTurn));
            
                    // Minimum turn power
                    if (Math.abs(autoTurn) < MIN_TURN) {
                        autoTurn = Math.signum(autoTurn) * MIN_TURN;
                    }
                }
            }
            
            // Smooth turn output
            autoTurn = SMOOTHING * autoTurn + (1.0 - SMOOTHING) * lastAutoTurn;
            lastAutoTurn = autoTurn;
            
            // Blend manual + auto
            double pa = manualTurn + autoTurn;
            pa = Math.max(-1.0, Math.min(1.0, pa));

            // ====================================================


            
            // BODY MOVEMENT
            double px = gamepad1.left_stick_x; //The power of the left stick on the x axis
                                                // Left to Right, -1.00 to 1.00
            double py = -gamepad1.left_stick_y; //The power of the left stick on the y axis
                                                // Down to Up, -1.00 to 1.00
            //math equation stuff...
            
            //p1 correlates to m1
            //p2 correlates to m2
            // etc.
            double p1 = -px + py - pa; //bl
            double p2 = px + py + pa; //br
            double p3 = -px + py + pa; //fr
            double p4 = px + py - pa; //fl
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            
            
            m1.setPower(p1);
            m2.setPower(p2); 
            m3.setPower(p3);
            m4.setPower(p4);
            
            
  
           // === Fine Power Adjustment For Launching - Single press only ===
            // increases/decreases by increments of 0.2
            boolean yPressed = gamepad2.y;
            boolean xPressed = gamepad2.x;
            
            if (yPressed && !yLastPressed) {  // Button just went down
                if (farRange) {
                    farPower = Math.min(1, farPower + 0.02);
                } else {
                    closePower = Math.min(1, closePower + 0.02);
                }
            }
            
            if (xPressed && !xLastPressed) {  // Button just went down
                if (farRange) {
                    farPower = Math.max(0.04, farPower - 0.02);
                } else {
                    closePower = Math.max(0.04, closePower - 0.02);
                }
            }
            
            yLastPressed = yPressed;
            xLastPressed = xPressed;
            
                        
            
            double goalBearing = 0;
            if (id20 != null) {
                goalBearing = id20.ftcPose.bearing;
            }else if (id24 != null) {
                goalBearing = id24.ftcPose.bearing;
            }
          
            
            //Setting mode either close or far
            if (gamepad2.left_bumper){
                farRange = false; //close range mode
                
            }else if(gamepad2.right_bumper){
                farRange = true;  //far range mode
            }
            
            
            //Launching
            if (gamepad2.right_trigger > 0) {
                
                double targetPower;
                
                if(farRange){
                    targetPower = farPower;
                }else{
                    targetPower = closePower;
                }
                

                leftThrow.setPower(targetPower);
                rightThrow.setPower(targetPower);
            } else {
                leftThrow.setPower(0);
                rightThrow.setPower(0);
            }
                        
            
            //Trigger
            if(gamepad2.a){
                trigger.setPosition(0.28);
                runTime.reset();
            }else if(runTime.seconds() > 0.5 ){
                trigger.setPosition(0.6);
            }
            
    
            //Feeder Forward
            //Top Feeder
            if (gamepad2.right_stick_y != 0){
                double feedPower = gamepad2.right_stick_y;
            
                topFeeder.setPower(feedPower);
            }else {
                double feedPower = 0;
            
                topFeeder.setPower(feedPower);
            }
            
            //Bottom Feeder
            if (gamepad2.left_stick_y != 0){
                double feedPower = -gamepad2.left_stick_y;
            
                bottomFeeder.setPower(feedPower);
            }else {
                double feedPower = 0;
            
                bottomFeeder.setPower(feedPower);
            }
            
           // double distanceMeters = topFeederSensor.getDistance(DistanceUnit.METER);
        
            
            //ID 20 Telemetry from WebCam
                //aprilTagWebcam.displayDetectionTelemetry(id20);
            
            
            
            //Display Positions of the motors
            telemetry.addData("Motor Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
                    
            //Display current throwing mode
            if(farRange == true){
                telemetry.addData("Far Range Mode Activated", "");
            }else{
                 telemetry.addData("Close Range Mode Activated", "");
            }
            
            telemetry.addLine();

            
            //Display far and close power values
            telemetry.addData("Close Motor Power", "%.2f", closePower);
            telemetry.addData("Far Motor Power", "%.2f", farPower);
            
            telemetry.update();
            
            telemetry.addLine();

            telemetry.addData("Y pressed", gamepad1.y);
            telemetry.addData("Tag Found", id20 != null || id24 != null);
            if (id20 != null) {
                telemetry.addData("Bearing", "%.2f", id20.ftcPose.bearing);
            }else if(id24 !=null){
                telemetry.addData("Bearing", "%.2f", id24.ftcPose.bearing);
            }
            telemetry.addData("autoTurn", "%.2f", autoTurn);
        }
        //Stops all motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
