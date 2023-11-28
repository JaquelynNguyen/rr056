
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.vision.BlueAlliancePipeline;
import org.firstinspires.ftc.teamcode.vision.RedAlliancePipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueEncoderClose", preselectTeleOp = "ActualTeleOp")
//@Disabled
public class BlueEncoderClose extends LinearOpMode {
    //Time
    private ElapsedTime runtime = new ElapsedTime();

    //Wheels
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;

    double target, _correction;
    private IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .5, correction;
    private ElapsedTime clawtime = new ElapsedTime();


    private CRServo arm_left;
    private CRServo arm_right;
    private CRServo v4bTurn;
    private Servo dropbox;
    private Servo rightFoldServo;
    private Servo leftFoldServo;
    //Cameras
    OpenCvCamera camera;

    static final double FEET_PER_METER = 3.28084;



    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //6:1
    //16:2
    //12:3
    int LeftTag = 6; // Tag ID 18 from the 36h11 family
    int RightTag = 12;
    int MiddleTag = 16;
    int location = 0;

    AprilTagDetection tagOfInterest = null;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {
        //Gets Hardware
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //Constant Variables
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        //SetWheelsToZero
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderFunction();


        int leftSpike = 1;
        int midSpike = 2;
        int rightSpike = 3;
        int propPosition = leftSpike;
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        //PlayButton

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BlueAlliancePipeline detector = new BlueAlliancePipeline(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */




        waitForStart(); //1100, 420, 900 for slides

        if (opModeIsActive() && !isStopRequested()) {
            switch (detector.getLocation()) {
                case LEFT:
                    propPosition = leftSpike;
                    break;
                case MID:
                    propPosition = midSpike;
                    break;
                case RIGHT:
                    propPosition = rightSpike;
                    break;
            }


            camera.stopStreaming();
            if (propPosition == leftSpike) {
                resetAngle();
                resetAngle();
                encoderFunction();
                runtime.reset();
                //move towards pole
                target = 20;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.4 - correction);
                    back_right.setPower(0.4 + correction);
                    front_left.setPower(0.4 - correction);
                    front_right.setPower(0.4 + correction);
                }
                encoderFunction();
                setPZero();
                rotate(-86, 0.45);
                encoderFunction();
                setPZero();
                target = 15;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.4 - correction);
                    back_right.setPower(0.4 + correction);
                    front_left.setPower(0.4 - correction);
                    front_right.setPower(0.4 + correction);
                }
                encoderFunction();
                setPZero();

                rotate(86, 0.45);
                encoderFunction();
                setPZero();
                target = 5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.4 - correction);
                    back_right.setPower(0.4 + correction);
                    front_left.setPower(0.4 - correction);
                    front_right.setPower(0.4 + correction);
                }
                encoderFunction();
                setPZero();


                target = 30;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                setPZero();
                encoderFunction();
                //move forward and knowck signal sleeve out of way

                //21.5 / 24

                rotate(-86, 0.45);
                encoderFunction();
                target = 27;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                lift.setTarget(1125);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                }
                arm.setPosition(Arm.Presets.OUTTAKE_POSITION);

                setPZero();
                encoderFunction();
                //move back a bit

                target = 10;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                runtime.reset();
                arm.setDropping(true);
                while (runtime.seconds() < 1.5) {
                    arm.update();
                }
                setPZero();
                encoderFunction();
                //move back a bit

                target = 10;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition())* 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                setPZero();
                encoderFunction();

                arm.setDropping(false);
                lift.setTarget(0);
                arm.setPosition(Arm.Presets.INTAKE_POSITION);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                }
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                    arm.update();
                }
                //move back a bit

                target = 40;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.35 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.5 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                setPZero();
                encoderFunction();
                //move back a bit



            } else if (propPosition == midSpike) {
                resetAngle();
                resetAngle();
                encoderFunction();
                runtime.reset();

                target = 40;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.44 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.44 - correction);
                    front_right.setPower(0.45 + correction);
                }
                encoderFunction();
                setPZero();
                target = 10;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5    - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                encoderFunction();
                setPZero();

                rotate(-82, 0.45);
                encoderFunction();
                target = 42;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                lift.setTarget(1100);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                }
                arm.setPosition(Arm.Presets.OUTTAKE_POSITION);

                setPZero();
                encoderFunction();
                //move back a bit

                target = 7;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                runtime.reset();
                arm.setDropping(true);
                while (runtime.seconds() < 1.5) {
                    arm.update();
                }
                arm.setDropping(false);
                while (runtime.seconds() < 1.5) {
                    arm.update();
                }
                setPZero();
                encoderFunction();
                //move back a bit

                target = 10;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                setPZero();
                encoderFunction();
                arm.setPosition(Arm.Presets.INTAKE_POSITION);
                runtime.reset();
                lift.setTarget(0);
                while (runtime.seconds() < 1.5) {
                    lift.update();
                }
                //move back a bit

                target = 26;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.35 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.5 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                setPZero();
                encoderFunction();
                //move back a bit




            } else {

                resetAngle();
                resetAngle();
                encoderFunction();
                runtime.reset();
                //move towards pole

                target = 5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(-0.43 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(0.47 + correction);
                }
                encoderFunction();
                setPZero();

                target = 38.5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.48 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.48 + correction);
                }
                encoderFunction();
                setPZero();

                //rightFoldServo.setPosition(-1);
                rotate(86, 0.45);
                encoderFunction();
                setPZero();

                target = 8;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                encoderFunction();
                setPZero();

                target = 7;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                encoderFunction();
                setPZero();
                rotate(-86, 0.45);
                encoderFunction();
                target = 8;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.25 - correction);
                    back_right.setPower(0.25 + correction);
                    front_left.setPower(0.25 - correction);
                    front_right.setPower(0.25 + correction);
                }
                encoderFunction();
                setPZero();
                rotate(-86, 0.45);

                encoderFunction();
                setPZero();

                target = 45;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                encoderFunction();
                setPZero();

                lift.setTarget(1050);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                }
                arm.setPosition(Arm.Presets.OUTTAKE_POSITION);

                setPZero();
                encoderFunction();
                //move back a bit

                target = 17.5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition())* 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                runtime.reset();
                arm.setDropping(true);
                while (runtime.seconds() < 1.5) {
                    arm.update();
                }
                arm.setDropping(false);
                runtime.reset();
                while (runtime.seconds() < 0.5) {
                    arm.update();
                }
                runtime.reset();
                lift.setTarget(1200);
                while (runtime.seconds() < 1.5){
                    lift.update();
                }
                setPZero();
                encoderFunction();
                //move back a bit

                target = 15;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                arm.setPosition(Arm.Presets.INTAKE_POSITION);
                setPZero();
                encoderFunction();
                //move back a bit
                lift.setTarget(0);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                }
                /*
                resetAngle();
                resetAngle();
                encoderFunction();
                runtime.reset();
                //move towards pole

                target = 5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(-0.43 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(0.47 + correction);
                }
                encoderFunction();
                setPZero();

                target = 34;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.48 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.48 + correction);
                }
                encoderFunction();
                setPZero();

                target = 25.5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.43 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.47 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                encoderFunction();
                setPZero();

                target = 7;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                encoderFunction();
                setPZero();

                target = 30;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.4 - correction);
                    back_right.setPower(-0.42 + correction);
                    front_left.setPower(-0.5 - correction);
                    front_right.setPower(0.48 + correction);
                }
                encoderFunction();
                setPZero();

                target = 14;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                encoderFunction();
                setPZero();
                //rightFoldServo.setPosition(-1);
                rotate(-86, 0.45);
                target = 50;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                lift.setTarget(1125);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                }
                arm.setPosition(Arm.Presets.OUTTAKE_POSITION);

                setPZero();
                encoderFunction();
                //move back a bit

                target = 17.5;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition())* 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(0.45 - correction);
                    back_right.setPower(0.45 + correction);
                    front_left.setPower(0.45 - correction);
                    front_right.setPower(0.45 + correction);
                }
                setPZero();
                encoderFunction();
                runtime.reset();
                arm.setDropping(true);
                while (runtime.seconds() < 1.5) {
                    arm.update();
                }
                arm.setDropping(false);
                runtime.reset();
                while (runtime.seconds() < 0.5) {
                    arm.update();
                }
                setPZero();
                encoderFunction();
                //move back a bit

                target = 15;
                while ((Math.abs(Math.abs(back_left.getCurrentPosition())  * 24 / 21.5  - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH ) > 40) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) * 24 / 21.5   - target * COUNTS_PER_INCH) > 40) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) * 24 / 21.5  - target * COUNTS_PER_INCH ) > 40) &&
                        opModeIsActive()) {
                    correction = checkDirection();
                    back_left.setPower(-0.45 - correction);
                    back_right.setPower(-0.45 + correction);
                    front_left.setPower(-0.45 - correction);
                    front_right.setPower(-0.45 + correction);
                }
                arm.setPosition(Arm.Presets.INTAKE_POSITION);
                setPZero();
                encoderFunction();
                //move back a bit
                lift.setTarget(0);
                runtime.reset();
                while(runtime.seconds() < 1.5) {
                    lift.update();
                }
                 */

            }

        }
    }


    /* Update the telemetry */


    public void encoderFunction() {

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Worked", 0);
    }

    public void Turnoff() {
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setPZero() {
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            newLeftTarget = back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * 30.0 / 30.75);
            newRightTarget = back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH * 30.0 / 30.75);
            back_left.setTargetPosition(newLeftTarget);
            front_left.setTargetPosition(newLeftTarget);
            back_right.setTargetPosition(newRightTarget);
            front_right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

        }
    }

    public void encoderStrafe(double speed,
                              double bLInches, double bRInches, double fLInches, double fRInches,
                              double timeoutS) {
        int newBLeftTarget;
        int newBRightTarget;
        int newFLeftTarget;
        int newFRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            newBLeftTarget = back_left.getCurrentPosition() + (int)(bLInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newBRightTarget = back_right.getCurrentPosition() + (int)(bRInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newFLeftTarget = back_left.getCurrentPosition() + (int)(fLInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newFRightTarget = back_right.getCurrentPosition() + (int)(fRInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            back_left.setTargetPosition(newBLeftTarget);
            front_left.setTargetPosition(newFLeftTarget);
            back_right.setTargetPosition(newBRightTarget);
            front_right.setTargetPosition(newFRightTarget);

            // Turn On RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (back_left.isBusy() && back_right.isBusy() && front_left.isBusy() && front_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBLeftTarget,  newBRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        back_left.getCurrentPosition(), back_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            back_left.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);
            front_right.setPower(0);

            // Turn off RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }

    public void resetAngle() {
        //Intrinsic is rotation to the robot
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .003;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate -left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        Turnoff();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        while (Math.abs(Math.abs(degrees) - Math.abs(getAngle())) > 5) {
            if (degrees - getAngle() < 0) {   // turn right.
                leftPower = power;
                rightPower = -power;
            } else if (degrees - getAngle() > 0) {   // turn left.
                leftPower = -power;
                rightPower = power;
            } else return;

            // set power to rotate.-
            back_left.setPower(leftPower);
            back_right.setPower(rightPower);
            front_left.setPower(leftPower);
            front_right.setPower(rightPower);
            telemetry.addData("deg:", degrees);
            telemetry.addData("angle", getAngle());
            telemetry.addData("math: ", Math.abs(degrees - getAngle()));
            telemetry.update();
        }

/*

        // rotate -until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && Math.abs(getAngle() - degrees) > 5) {
            }
        } else    // left turn.
            while (opModeIsActive() && Math.abs(getAngle() - degrees) > 5) {
            }

 */

        // turn the motors off.
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);

        // wait for rotation to stop.
        sleep(150);

        // reset angle tracking on new heading.
        resetAngle();
    }
}