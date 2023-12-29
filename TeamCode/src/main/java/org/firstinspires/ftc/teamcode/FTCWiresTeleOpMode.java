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

    @Override
    public void runOpMode() throws InterruptedException {
        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","8755");
        telemetry.update();


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            drive.setGamePad1(gamepad1);
            drive.setGamePad2(gamepad2);

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
                telemetry.addLine("Arm Location:");
                telemetry.addData("", drive.liftArm.getCurrentPosition());
                telemetry.addLine("Outtake Rotate Servo Position");
                telemetry.addData("Position:", drive.outtakeRotate.getPosition());
;               telemetry.update();

                drive.liftArmAdjust();

                if (gamepad1.a) {
                    drive.intake();
                }

                else if (gamepad1.b){
                    drive.reverseIntake();
                }

                else {
                    drive.idleIntake();
                }
                //Boost function
                if (gamepad1.right_bumper && SLOW_DOWN_FACTOR <= 0.7){
                    SLOW_DOWN_FACTOR = 1;
                }
                //Return to normal speed
                else  {
                    SLOW_DOWN_FACTOR = 0.6;
                }

                if (gamepad1.dpad_down){
                    drive.releasePixels();
                }

                if (gamepad1.y){
                    drive.rotateOuttakeToOuttakePos();
                }

                if (gamepad1.x){
                    drive.moveToIntakePos = true;
                }

                if (gamepad1.left_bumper){
                    drive.rotateOuttakeToIntakePos();
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