// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class DaveTrapezoid {

    public static class Constraints {
        public double vel, acc;

        public Constraints(double vel, double acc) {
            this.vel = vel;
            this.acc = acc;
        }

        @Override
        public String toString() {
            return "vel: " + vel + ", acc: " + acc;
        }

        public Constraints copy() {
            return new Constraints(vel, acc);
        }
    }

    public static class State {
        public double pos;

        public double vel;

        public State() {
        }

        public State(double position, double velocity) {
            this.pos = position;
            this.vel = velocity;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof State) {
                State rhs = (State) other;
                return this.pos == rhs.pos && this.vel == rhs.vel;
            } else {
                return false;
            }
        }
    }

    private boolean forwards;
    private Constraints cons;
    private double p0, p1, v0, v1;
    private double tA, tV, tD;

    public DaveTrapezoid(Constraints constraints, State start, State end) {
        forwards = end.pos >= start.pos;
        cons = constraints;

        p0 = forwards ? start.pos : end.pos;
        p1 = forwards ? end.pos : start.pos;
        v0 = forwards ? start.vel : -end.vel;
        v1 = forwards ? end.vel : -start.vel;

        if (v0 > cons.vel) {
            v0 = cons.vel;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = v0 / cons.acc;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * cons.acc / 2.0;

        double cutoffEnd = v1 / cons.acc;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * cons.acc / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (p1 - p0) + cutoffDistEnd;
        double accelerationTime = cons.vel / cons.acc;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * cons.acc;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / cons.acc);
            fullSpeedDist = 0;
        }

        tA = accelerationTime - cutoffBegin;
        tV = fullSpeedDist / cons.vel;
        tD = accelerationTime - cutoffEnd;
    }

    public double totalTime() {
        return tA + tV + tD;
    }

    public State calculate(double t) {
        State result = new State(p0, v0);
        double tt = totalTime();
        if (!forwards)
            t = tt - t;

        if (t < 0.0) {
            result.pos = p0;
            result.vel = v0;
        } else if (t < tA) {
            result.vel += t * cons.acc;
            result.pos += (v0 + t * cons.acc / 2.0) * t;
        } else if (t < tA + tV) {
            result.vel = cons.vel;
            result.pos += (v0 + tA * cons.acc / 2.0) * tA + cons.vel * (t - tA);
        } else if (t <= tt) {
            double tl = tt - t;
            result.vel = v1 + (tl) * cons.acc;
            result.pos = p1 - (v1 + tl * cons.acc / 2.0) * tl;
        } else {
            result.pos = p1;
            result.vel = v1;
        }

        return result;
    }

    public double timeLeftUntil(double target) {
        double position = p0;
        double velocity = v0;

        double endAccel = tA;
        double endFullSpeed = tV;

        if (target < position) {
            endAccel = -endAccel;
            endFullSpeed = -endFullSpeed;
            velocity = -velocity;
        }

        endAccel = Math.max(endAccel, 0);
        endFullSpeed = Math.max(endFullSpeed, 0);

        final double acceleration = cons.acc;
        final double decceleration = -cons.acc;

        double distToTarget = Math.abs(target - position);
        if (distToTarget < 1e-6) {
            return 0;
        }

        double accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

        double deccelVelocity;
        if (endAccel > 0) {
            deccelVelocity = Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accelDist));
        } else {
            deccelVelocity = velocity;
        }

        double fullSpeedDist = cons.vel * endFullSpeed;
        double deccelDist;

        if (accelDist > distToTarget) {
            accelDist = distToTarget;
            fullSpeedDist = 0;
            deccelDist = 0;
        } else if (accelDist + fullSpeedDist > distToTarget) {
            fullSpeedDist = distToTarget - accelDist;
            deccelDist = 0;
        } else {
            deccelDist = distToTarget - fullSpeedDist - accelDist;
        }

        double accelTime = (-velocity + Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accelDist)))
                / acceleration;

        double deccelTime = (-deccelVelocity
                + Math.sqrt(Math.abs(deccelVelocity * deccelVelocity + 2 * decceleration * deccelDist)))
                / decceleration;

        double fullSpeedTime = fullSpeedDist / cons.vel;

        return forwards ? accelTime + fullSpeedTime + deccelTime
                : totalTime() - (accelTime + fullSpeedTime + deccelTime);
    }
}
