package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * FTC WIRES TeleOp Example
 *
 */

@TeleOp(name = "FTC Wires TeleOp", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {
    public DcMotor intakeMotor;
    public DcMotor liftArm;
    public CRServo counterRoller;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        liftArm = hardwareMap.get(DcMotor.class, "liftArm");
        counterRoller = hardwareMap.get(CRServo.class, "roller");
        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","8755 E-STORM");
        telemetry.update();


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","8755");
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                //x & y flipped weirdly for some reason in this code - x is from outtake to intake and y from side to side
                                gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                gamepad1.right_trigger * SLOW_DOWN_FACTOR -gamepad1.left_trigger * SLOW_DOWN_FACTOR

                        ),
                        //Can change the below code to be "gamepad1.right_stick_x" for separate turning and driving
                        gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                ));

                drive.updatePoseEstimate();

                /*telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                /telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                /telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                /telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());*/

                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();
                //Intake function
                if (gamepad1.a) {
                    counterRoller.setPower(1);
                    intakeMotor.setPower(1);
                }
                else if(gamepad1.b){
                    counterRoller.setPower(-0.85);
                    intakeMotor.setPower(-0.75);
                }
                else{
                    counterRoller.setPower(0);
                    intakeMotor.setPower(0);
                }
                //Boost function
                if (gamepad1.y && SLOW_DOWN_FACTOR <= 0.7){
                    SLOW_DOWN_FACTOR = SLOW_DOWN_FACTOR + 0.2;
                }
                else{
                    SLOW_DOWN_FACTOR = 0.5;
                }
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                -gamepad1.left_trigger * SLOW_DOWN_FACTOR
                                        -gamepad1.left_stick_x * SLOW_DOWN_FACTOR

                        ),
                        -gamepad1.right_stick_y * SLOW_DOWN_FACTOR
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
    }
}