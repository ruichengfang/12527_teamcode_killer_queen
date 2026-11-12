package org.firstinspires.ftc.teamcode.command;

/** 将副操方向键的按下边沿转换为 X/Y 增量方向。 */
public final class PinpointTrimButtons {
    private boolean previousUp, previousDown, previousLeft, previousRight;
    private int xDirection, yDirection;

    public void update(boolean up, boolean down, boolean left, boolean right) {
        xDirection = (up && !previousUp ? 1 : 0) - (down && !previousDown ? 1 : 0);
        yDirection = (right && !previousRight ? 1 : 0) - (left && !previousLeft ? 1 : 0);
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
    }

    public int getXDirection() { return xDirection; }
    public int getYDirection() { return yDirection; }
}
