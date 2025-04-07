package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "motor test")
@Config
public class day0test extends LinearOpMode {
    public Servo sWristSpin;
    public Servo sClaw;
    public Servo sWrist;
    public Servo sLiftClaw;
    public Servo sLiftArm;
    public Servo sLiftWrist;

    public DcMotorEx mLeftBack;
    public DcMotorEx mRightBack;
    public DcMotorEx mLeftFront;
    public DcMotorEx mRightFront;

    public DcMotorEx mFrontArm;
    public DcMotorEx mLiftUp;
    public DcMotorEx mListDown;

    @Override
    public void runOpMode() {
        sWristSpin = hardwareMap.get(Servo.class, "wristSpin");
        sClaw = hardwareMap.get(Servo.class, "claw");
        sWrist = hardwareMap.get(Servo.class, "wrist");
        sLiftClaw = hardwareMap.get(Servo.class, "liftClaw");
        sLiftArm = hardwareMap.get(Servo.class, "liftArm");
        sLiftWrist = hardwareMap.get(Servo.class, "liftWrist");

        mLeftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        mRightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        mLeftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        mRightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        mFrontArm = hardwareMap.get(DcMotorEx.class, "frontArm");
        mLiftUp = hardwareMap.get(DcMotorEx.class, "liftUp");
        mListDown = hardwareMap.get(DcMotorEx.class, "liftDown");


    }
}
