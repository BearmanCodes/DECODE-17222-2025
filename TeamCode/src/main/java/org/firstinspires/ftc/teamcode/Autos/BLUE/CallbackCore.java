package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.callbacks.PathCallback;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ShooterAutoCore;
import org.firstinspires.ftc.teamcode.Temporary.ModeCore;

public class CallbackCore implements PathCallback {
    private boolean firstTime = true;

    private ShooterAutoCore shooterCore;

    private Follower follower;

    private Telemetry telemetry;

    private boolean runIntakeAfter;

    private int nextFlyVel;

    private int nextFryVel;

    private int pathIndex;

    CallbackCore(Follower follow, ShooterAutoCore shooterAutoCore, Telemetry tele){
        this.shooterCore = shooterAutoCore;
        this.telemetry = tele;
        this.follower = follow;
    }
    public CallbackCore setRunIntakeAfter(boolean argRunIntakeAfter){
        this.runIntakeAfter = argRunIntakeAfter;
        return this;
    }

    public CallbackCore setNextFlySpeeds(int argFlyVel, int argFryVel) {
        this.nextFlyVel = argFlyVel;
        this.nextFryVel = argFryVel;
        return this;
    }

    public CallbackCore setPathIndex(int argPathIndex) {
        this.pathIndex = argPathIndex;
        return this;
    }

    @Override
    public boolean run() {
        follower.pausePathFollowing();
        while(!shooterCore.shoot(3, telemetry)){
            telemetry.update();
        }
        shooterCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_LOAD);
        if (runIntakeAfter) shooterCore.in();
        shooterCore.spinUpFlys(nextFlyVel, nextFryVel);
        follower.resumePathFollowing();
        return true;
    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    public void initialize() {
        if (firstTime){
            ShooterAutoCore.failsafeTimer.reset();
            shooterCore.setCRPower(1, telemetry);
            shooterCore.luigiServo.setPosition(ModeCore.LUIGI_HOPPER_SHOOT);
            firstTime = false;
        }
    }

    @Override
    public int getPathIndex() {
        return pathIndex;
    }
}
