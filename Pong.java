import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.utility.TextMenu;

import java.io.File;
import java.util.Random;

import static java.lang.Math.*;

class Ball {
    private double vel;
    private double angle;
    private final float planeLengthX; // Tacho X-direction:	940 (32.76cm)	28.7 Tacho / cm
    private final float planeLengthY; // Tacho Y-direction:	856 (28.08cm)	30.5 Tacho / cm
    private final EV3LargeRegulatedMotor motorX;
    private final EV3LargeRegulatedMotor motorY;
    private float tachoPrCmX;
    private float tachoPrCmY;

    public Ball(float planeLengthX, float planeLengthY,
                EV3LargeRegulatedMotor motorX, EV3LargeRegulatedMotor motorY) {
        this.planeLengthX = planeLengthX;
        this.planeLengthY = planeLengthY;
        this.motorX = motorX;
        this.motorY = motorY;
    }

    public int getXPos() {
        return motorX.getTachoCount();
    }

    public int getYPos() {
        return motorY.getTachoCount();
    }

    private double getXVel() {
        return vel * cos(angleInRad());
    }

    private double getYVel() {
        return vel * sin(angleInRad());
    }

    public double getVel() {
        return vel;
    }

    private double angleInRad() {
        return PI * angle / 180;
    }

    public float getTPCX() {
        return tachoPrCmX;
    }

    public float getTPCY() {
        return tachoPrCmY;
    }

    private void setXPos(int newXPos) {
        motorX.setSpeed(150);
        motorX.rotateTo(newXPos, true);
    }

    private void setYPos(int newYPos) {
        motorY.setSpeed(150);
        motorY.rotateTo(newYPos, true);
    }

    public void setVel(double vel) {
        this.vel = vel;
        long xVel = round(getXVel());
        long yVel = round(getYVel());

		/* Motor backward if speed is negative, forward otherwise */
        motorX.setSpeed(xVel);
        if (xVel < 0) {
            motorX.backward();
        } else {
            motorX.forward();
        }

        motorY.setSpeed(yVel);
        if (yVel < 0) {
            motorY.backward();
        } else {
            motorY.forward();
        }
    }

    private void setAngle(double angle) {
        this.angle = angle;
    }

    public void hitWall() {
        setAngle(360 - angle);
        setVel(vel);
    }

    public void hitPad(double posOnPad, double padLength) {
        double newAngle = (-70 * (1 - posOnPad / padLength) + 70 * posOnPad / padLength);
        boolean isRight = (cos(angleInRad()) > 0);

        setAngle(isRight ? (180 - newAngle) : newAngle);
    }


    private void stopWhenStalled() {
        boolean x = true;
        boolean y = true;

        while (x || y) {
            if (motorX.isStalled()) {
                motorX.stop();
                x = false;
            }
            if (motorY.isStalled()) {
                motorY.stop();
                y = false;
            }
        }
    }

    public void calibrate() {
        motorX.setSpeed(50);
        motorY.setSpeed(50);
        motorX.setStallThreshold(5, 1);
        motorY.setStallThreshold(5, 1);

        motorX.backward();
        motorY.backward();

        stopWhenStalled();

        motorX.flt();
        motorY.flt();

        Delay.msDelay(500);
        motorX.resetTachoCount();
        motorY.resetTachoCount();
        motorX.stop();
        motorY.stop();
        Delay.msDelay(1000);

        motorX.forward();
        motorY.forward();

        Delay.msDelay(3000);
        motorX.setStallThreshold(2, 1);
        motorY.setStallThreshold(2, 1);

        stopWhenStalled();

        motorX.flt();
        motorY.flt();

        Delay.msDelay(500);
        motorX.stop();
        motorY.stop();
        tachoPrCmX = getXPos() / planeLengthX;
        tachoPrCmY = getYPos() / planeLengthY;
        motorX.setStallThreshold(50, 50);
        motorY.setStallThreshold(50, 50);
        Delay.msDelay(100);
    } // end method

    public void goToMiddle() {
        int middleX = round(planeLengthX * tachoPrCmX / 2); // Length to center in tacho for X-axis
        int middleY = round(planeLengthY * tachoPrCmY / 2); // Length to center in tacho for Y-axis

        motorX.stop();
        motorY.stop();

        setXPos(middleX);
        setYPos(middleY);

		/* Wait for ball to reach the middle */
        boolean x = true;
        boolean y = true;
        while (x || y) {
            if (getXPos() >= middleX - 2 && getXPos() <= middleX + 2) {
                x = false;
            }
            if (getYPos() >= middleY - 2 && getYPos() <= middleY + 2) {
                y = false;
            }
        }
    } // end method

    public void kickOff(int scorer) {
        Random random = new Random();

		/* Selects a side based on who scored */
        int direction;
        switch (scorer) {
            case 0:
                direction = random.nextInt(2) * 180;
                break;
            case 1:
                direction = 180;
                break;
            case 2:
                direction = 0;
                break;
            default:
                throw new IllegalArgumentException("Illegal input value.");
        }

		/* Selects a random angle between 70 and -70 towards the side selected*/
        double maxAngle = direction + 70;
        double minAngle = direction - 70;
        double startAngle;
        do {
            startAngle = random.nextGaussian() * 30 + direction;
        } while (startAngle > maxAngle || startAngle < minAngle);

        setAngle(startAngle);
    }
} // end class


class Pad {
    private float pos;
    private final float width;
    private final EV3UltrasonicSensor ultraSensor;
    private final SampleProvider playerRead;
    private final float[] playerSample;
    private final float tachoPrCm;

    public Pad(float width, EV3UltrasonicSensor ultraSensor,
               float tachoPrCm) {
        this.pos = 0;
        this.tachoPrCm = tachoPrCm;
        this.width = width * tachoPrCm;
        this.ultraSensor = ultraSensor;
        this.playerRead = this.ultraSensor.getDistanceMode();
        this.playerSample = new float[this.playerRead.sampleSize()];
    }

    public float getPos() {
		/* Reads the pad position from sensor */
        playerRead.fetchSample(playerSample, 0);
        pos = (playerSample[0] - 0.035f) * tachoPrCm * 100;
        return pos;
    }

    public float getWidth() {
        return width;
    }
} // end class


class Player {
    private int score;
    private final Pad pad;

    public Player(Pad pad) {
        this.score = 0;
        this.pad = pad;
    }

    public int getScore() {
        return score;
    }

    public double getWidth() {
        return pad.getWidth();
    }

    public double getPos() {
        return pad.getPos();
    }

    public void addPoint() {
        score++;
    }

    public void resetScore(){
        score = 0;
    }
} // end class


class Speaker extends Thread{
    private static int freq; // Hertz
    private static final int VOL = 100; // Percent
    private static String file;
    private static int duration; // Milliseconds
    private static int choice;

  //  public Speaker() { }

    public void playSound(String newFile) {
        file = newFile;
        choice = 1;
    }

    public static void playSound(int newFreq, int newDuration) {
        freq = newFreq;
        duration = newDuration;
        choice = 2;
    }

    public static void countdownSound() {
        choice = 3;
    }

    private static void runCountdownSound() {
        Sound.playTone(300, 150);
        Delay.msDelay(700);
        Sound.playTone(300, 150);
        Delay.msDelay(700);
        Sound.playTone(300, 150);
        Delay.msDelay(700);
        Sound.playTone(1000, 500);
    }

    public static void missSound() {
        choice = 4;
    }

    private static void runMissSound() {
        Sound.playTone(0, 50);
        Delay.msDelay(150);
        Sound.playTone(0, 50);
        Delay.msDelay(150);
        Sound.playTone(0, 500);
    }

    public static void victorySound() {
        choice = 5;
    }

    private static void runVictorySound() {
        Sound.playTone(400, 150);
        Delay.msDelay(50);
        Sound.playTone(400, 500);
    }

    public void run() {
        loop:
        while (true) {
            switch (choice) {
                case 0:
                    Delay.msDelay(10);
                    break;
                case 1:
                    File playFile = new File(file);
                    Sound.playSample(playFile, VOL);
                    Delay.msDelay(50);
                    choice = 0;
                    break;
                case 2:
                    Sound.playTone(freq, duration, VOL);
                    choice = 0;
                    break;
                case 3:
                    runCountdownSound();
                    choice = 0;
                    break;
                case 4:
                    runMissSound();
                    choice = 0;
                    break;
                case 5:
                    runVictorySound();
                    choice = 0;
                    break;
                default:
                    System.out.println("Error at line " + Thread.currentThread().getStackTrace()[0].getLineNumber());
                    choice = 0;
                    break loop;
            } // switch
        } // loop
    }
}


class Screen extends Thread {
    private final GraphicsLCD lcd;
    private boolean runningAnimation = false;
    private int choice = 0;
    private String string;
    private int score1;
    private int score2;
    private final int HEIGHT = 128;
    private final int WIDTH = 178;

    public Screen (GraphicsLCD lcd) {
        this.lcd = lcd;
    }

    private void resetGraphics() {
        lcd.setColor(GraphicsLCD.BLACK);
        lcd.setFont(Font.getDefaultFont());
    }

    public void run() {
        loop:
        while (true) {
            switch (choice) {
                case 0:
                    break;
                case 1:
                    runStartupAnimation();
                    choice = 0;
                    break;
                case 2:
                    runPrintString();
                    choice = 0;
                    break;
                case 3:
                    runDisplayScore();
                    choice = 0;
                    break;
                default:
                    System.out.println("Error at line " + Thread.currentThread().getStackTrace()[0].getLineNumber());
                    break loop;
            }
        }
    } // end method

    private void runStartupAnimation() {
        lcd.clear();
        resetGraphics();

        lcd.setColor(GraphicsLCD.BLACK);
        for (int n = 0; n < WIDTH / 2; n += 2) {
            lcd.drawRect(WIDTH / 2 - n, HEIGHT / 2 - n, 2 * n, 2 * n);
            Delay.msDelay(50);
        }

        int p = 5;
        for (int i = 8; i <= 32; i *= 2) {
            lcd.setColor(GraphicsLCD.WHITE);
            int k = (i == 8) ? 5 : 6;
            lcd.fillRect((WIDTH - i * k) / 2 - p, HEIGHT / 2 - i / 2 - p, i * k + 2 * p, i + 2 * p);
            switch (i) {
                case 8:
                    lcd.setFont(Font.getSmallFont());
                    break;
                case 16:
                    lcd.setFont(Font.getDefaultFont());
                    break;
                case 32:
                    lcd.setFont(Font.getLargeFont());
                    break;
                default:
                    System.out.println("Error at line " + Thread.currentThread().getStackTrace()[0].getLineNumber());
                    break;
            }
            lcd.setColor(GraphicsLCD.BLACK);
            lcd.drawString("TAM-PONG", WIDTH / 2, HEIGHT / 2 - i / 2, GraphicsLCD.HCENTER);
            Delay.msDelay(1000);
        }

        lcd.clear();
        resetGraphics();

        lcd.drawString("TAM-PONG", 89, 0, GraphicsLCD.HCENTER);

        int padH = 20;
        int padW = 5;

        lcd.fillRect(0, HEIGHT / 2 - padH / 2, padW, padH);
        lcd.fillRect(WIDTH - padW, HEIGHT / 2 - padH / 2, padW, padH);

        while (runningAnimation) {
            for (int i = 89; i > 4 && runningAnimation; i--) {
                lcd.setColor(GraphicsLCD.BLACK);
                lcd.fillRect(i, HEIGHT/2, 2, 2);
                Delay.msDelay(30);
                lcd.setColor(GraphicsLCD.WHITE);
                lcd.fillRect(i + 2, HEIGHT/2, 2, 2);
            }
            for (int j = 6; j < 172 && runningAnimation; j++) {
                lcd.setColor(GraphicsLCD.BLACK);
                lcd.fillRect(j, HEIGHT / 2, 2, 2);
                Delay.msDelay(30);
                lcd.setColor(GraphicsLCD.WHITE);
                if (j > 6) {
                    lcd.fillRect(j - 2, HEIGHT / 2, 2, 2);
                }
            }
            for (int k = 172; k > 89 && runningAnimation; k--) {
                lcd.setColor(GraphicsLCD.BLACK);
                lcd.fillRect(k, HEIGHT / 2, 2, 2);
                Delay.msDelay(30);
                lcd.setColor(GraphicsLCD.WHITE);
                if (k < 170) {
                    lcd.fillRect(k + 2, HEIGHT / 2, 2, 2);
                }
            }
        }

        lcd.clear();
    }

    public void startupAnimation() {
        runningAnimation = true;
        choice = 1;
    }

    public void stopAnimation() {
        runningAnimation = false;
    }

    private void runPrintString() {
        lcd.clear();
        resetGraphics();
        lcd.setFont(Font.getLargeFont());
        int s = 16;
        if (Font.getLargeFont().stringWidth(string) > WIDTH) {
            lcd.setFont(Font.getDefaultFont());
            s = 8;
            if (Font.getDefaultFont().stringWidth(string) > WIDTH) {
                lcd.setFont(Font.getSmallFont());
                s = 4;
            }
        }
        lcd.drawString(string, 89, 64 - s, GraphicsLCD.HCENTER);
        resetGraphics();
    }

    public void printString(String string) {
        this.string = string;
        choice = 2;
    }

    private void runDisplayScore() {
        lcd.clear();
        resetGraphics();
        lcd.drawString("TAM-PONG", 89, 0, GraphicsLCD.HCENTER);
        lcd.setFont(Font.getLargeFont());
        int pos = 20;
        lcd.drawString(Integer.toString(score1), 89 + 2 * pos, 48, GraphicsLCD.LEFT);
        lcd.drawString(Integer.toString(score2), 89 - 2 * pos, 48, GraphicsLCD.RIGHT);

        lcd.setStrokeStyle(GraphicsLCD.DOTTED);
        lcd.drawLine(89, 20, 89, 128);
    }

    public void displayScore(int score1, int score2) {
        this.score1 = score1;
        this.score2 = score2;
        choice = 3;
    }
}


class CheckForExit extends Thread {
    private final EV3TouchSensor touchSensor;
    // --Commented out by Inspection (22.11.2016 12.31):private final SampleProvider touchRead;
    private final float[] touchSample;

    public CheckForExit(EV3TouchSensor touchSensor) {
        this.touchSensor =  touchSensor;
        // --Commented out by inspection (22.11.2016 12:59):this.touchRead = this.touchSensor;
        this.touchSample = new float[this.touchSensor.sampleSize()];
    }

    public void run() {
		/* Exits program if touch sensor is pressed */
        while (true) {
            touchSensor.fetchSample(touchSample, 0);
            if (touchSample[0] > 0) {
                System.exit(0);
            }
        }
    }
} // end class


class Pong {
    public static void main(String[] args) throws Exception {
        final int MAX_SCORE = 5; // score
        final float PAD_WIDTH = 4.68f; // cm
        final float PLANE_X = 32.76f; // cm
        final float PLANE_Y = 28.08f; // cm
        final int BALL_SPEED = 150; // degrees / sec
        final int INCREMENT = 15; // degrees / sec - increase speed for every pad hit
        final int PAD_TOL = 10; // tacho
        final int WALL_TOL = 10; //tacho

		/* Define brick and get its ports */
        Brick brick = BrickFinder.getDefault();
        Port s1 = brick.getPort("S1"); // EV3 Ultrasonic sensor player 2
        Port s4 = brick.getPort("S4"); // EV3 Ultrasonic sensor plater 1
        Port s3 = brick.getPort("S3"); // EV3 Touch sensor
        Port a = brick.getPort("A"); // EV3 large motor X-axis
        Port b = brick.getPort("B"); // EV3 large motor Y-axis

		/* Get EV3 and gain access to display and screen */
        EV3 ev3 = (EV3) BrickFinder.getLocal();
        GraphicsLCD lcd = ev3.getGraphicsLCD();
        Keys keys = ev3.getKeys();

		/* Define touch sensor */
        EV3TouchSensor touchSensor = new EV3TouchSensor(s3);
        CheckForExit c = new CheckForExit(touchSensor);
        c.start();

		/* Define graphical screen */
        Screen screen = new Screen(lcd);

		/* Define speaker */
        Speaker speaker = new Speaker();

		/* Define UltraSonic sensors */
        EV3UltrasonicSensor p1Sensor;
        EV3UltrasonicSensor p2Sensor;
        while(true){
            try{
                p1Sensor = new EV3UltrasonicSensor(s4);
                p2Sensor = new EV3UltrasonicSensor(s1);
                break;
            } catch(IllegalArgumentException e){
                screen.printString("Invalid sensor mode");
            }
        }

        /* Define motors */
        EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(a);
        EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(b);

		/* Define ball */
        Ball ball = new Ball(PLANE_X, PLANE_Y, motorA, motorB);

        screen.start();
        speaker.start();

        screen.startupAnimation();

        ball.calibrate();

        screen.stopAnimation();

		/* Define pads */
        Pad pad1 = new Pad(PAD_WIDTH, p1Sensor, ball.getTPCY());
        Pad pad2 = new Pad(PAD_WIDTH, p2Sensor, ball.getTPCY());

		/* Define players */
        Player p1 = new Player(pad1);
        Player p2 = new Player(pad2);

        final float MAX_X_POS = PLANE_X * ball.getTPCX();
        final float MAX_Y_POS = PLANE_Y * ball.getTPCY();

        Delay.msDelay(500);
        match:
        do {
            ball.goToMiddle();

            screen.printString("Press to start");
            keys.waitForAnyPress();
            Delay.msDelay(500); // Separate key press sound from countdown

            Speaker.countdownSound();
            Delay.msDelay(2750);

            ball.kickOff(0);

            screen.displayScore(p1.getScore(), p2.getScore());

            int rounds = 0;

            gameLoop:
            while (true) {
                int speedIncrease = (BALL_SPEED * rounds * 2) / MAX_SCORE;
                ball.setVel(BALL_SPEED + speedIncrease);

                int lastTouched = 0; // Prevents the ball hitting the same element multiple times in a row
                int lastPad = 0; // Prevents the ball from getting stuck in corners


		    	/* Check ball position */
                ballLoop:
                while (true) {

                    boolean ballAtPlayer1Side = ((ball.getXPos() <= (PAD_TOL))
                            && (lastTouched != 1)
                            && (lastPad != 1));
                    boolean ballAtPlayer2Side = ((ball.getXPos() >= (MAX_X_POS - PAD_TOL))
                            && (lastTouched != 2)
                            && (lastPad != 2));
                    boolean ballAtUpperWall = ((ball.getYPos() <= (WALL_TOL))
                            && (lastTouched != 3));
                    boolean ballAtLowerWall = ((ball.getYPos() >= (MAX_Y_POS - WALL_TOL))
                            && (lastTouched != 4));


                    if (ballAtPlayer1Side) {
                        boolean ballHitsPad = (ball.getYPos() > (p1.getPos() - PAD_TOL)
                                && ball.getYPos() < (p1.getPos() + p1.getWidth() + PAD_TOL));
                        if (ballHitsPad) {
                            lastTouched = 1;
                            lastPad = 1;
                            Speaker.playSound(420, 50);
                            ball.hitPad(ball.getYPos() - p1.getPos(), p1.getWidth());
                            ball.setVel(ball.getVel() + INCREMENT);
                        } else {
                            p2.addPoint();
                            screen.displayScore(p1.getScore(), p2.getScore());
                            Speaker.missSound();
                            Delay.msDelay(200);
                            ball.goToMiddle();
                            Delay.msDelay(500);
                            Speaker.playSound(250,100);
                            ball.kickOff(2);
                            break ballLoop;
                        }
                    } else if (ballAtPlayer2Side) {
                        boolean ballHitsPad = (ball.getYPos() > (p2.getPos() - PAD_TOL)
                                && ball.getYPos() < (p2.getPos() + p2.getWidth() + PAD_TOL));
                        if (ballHitsPad) {
                            lastTouched = 2;
                            lastPad = 2;
                            ball.hitPad(ball.getYPos() - p2.getPos(), p2.getWidth());
                            ball.setVel(ball.getVel() + INCREMENT);
                            Speaker.playSound(420, 50);
                        } else {
                            p1.addPoint();
                            screen.displayScore(p1.getScore(), p2.getScore());
                            Speaker.missSound();
                            Delay.msDelay(200);
                            ball.goToMiddle();
                            Delay.msDelay(500);
                            ball.kickOff(1);
                            Speaker.playSound(250,100);
                            break ballLoop;
                        }
                    } else if (ballAtUpperWall) {
                        lastTouched = 3;
                        ball.hitWall();
                        Speaker.playSound(210, 50);
                    } else if (ballAtLowerWall) {
                        lastTouched = 4;
                        ball.hitWall();
                        Speaker.playSound(210, 50);
                    }
                } // end ball
                rounds++;

			    /* Check victory condition */
                if (p1.getScore() == MAX_SCORE || p2.getScore() == MAX_SCORE) {
                    String winner = (p1.getScore() == MAX_SCORE) ? "PLAYER 1 WON!" : "PLAYER 2 WON!";
                    screen.printString(winner);
                    Speaker.victorySound();
                    Delay.msDelay(4000);
                    screen.printString("");

                    TextMenu textmenu = new TextMenu(new String[]{"Yes", "No"}, 1, "Rematch?");
                    int selection = textmenu.select();
                    if (selection == 0) {
                        p1.resetScore();
                        p2.resetScore();
                        continue match;
                    } else {
                        break match;
                    }
                }
            } // end gameLoop
        } while (true);
        System.exit(0);
    } // end main
}