clearscreen.

// Create new passed in parameter for logfile, delete
DECLARE PARAMETER logfile.
LIST FILES IN fileList.
SET exists to FALSE.

FOR file IN fileList {
    IF file:NAME = logfile {
        set exists to TRUE.
    }
}

IF exists {
    PRINT logfile + " exists, deleting.".
    DELETE logfile.
} ELSE {
    PRINT logfile + " does not exist, creating new.".
}

// How long to run test for
DECLARE PARAMETER total_time. // Time to record data till stop & opening chutes

// Initial throttle is full till certain height
SET thrott TO 1.
SET dthrott TO 0.
SET dthrott_p TO 0.
SET dthrott_d TO 0.

// Calculate g-force
SET g TO KERBIN:MU / KERBIN:RADIUS^2.
LOCK accvec TO SHIP:SENSORS:ACC - SHIP:SENSORS:GRAV.
LOCK gforce TO accvec:MAG / g.
LOCK hover_throttle_level TO MIN(1, MAX(0, SHIP:MASS * g / MAX(0.0001, curr_engine:AVAILABLETHRUST))).


LOCK THROTTLE TO thrott.
LOCK STEERING TO R(0,0,-90) + HEADING(90,90).

// All running engines
LIST ENGINES IN all_engines.
// Use first (and only) for calculations (assuming ship with single engine)
SET curr_engine TO all_engines[0].

// Proportional gains
DECLARE PARAMETER Kp.
DECLARE PARAMETER Kd.
PRINT "Kp = " + Kp + ", Kd = " + Kd.

PRINT "LAUNCH".
SAS ON.

// If we've not yet gone to stage
IF STAGE:NUMBER = 2 {
  WAIT UNTIL STAGE:READY.
  STAGE.
}

curr_engine:ACTIVATE(). // Activate engine if it wasn't already

// Start PID Loop
SET goal_altitude TO 100.
LOCK dthrott_p TO Kp * (goal_altitude - SHIP:ALTITUDE).
LOCK dthrott_d TO Kd * (0 - SHIP:VERTICALSPEED).
LOCK dthrott TO dthrott_p + dthrott_d.

// Set up logging string
LOCK logline TO (TIME:SECONDS - start_time)
        + " " + SHIP:ALTITUDE
        + " " + SHIP:VERTICALSPEED
        + " " + SHIP:SENSORS:ACC:MAG
        + " " + gforce
        + " " + thrott
        + " " + dthrott_p
        + " " + dthrott_d.

SET t0 TO TIME:SECONDS.
SET t1 TO TIME:SECONDS.
SET start_time TO t0.

UNTIL TIME:SECONDS-start_time > total_time {
  SET dt TO TIME:SECONDS - t0.
  SET dt1 TO TIME:SECONDS - t1.
  
  // Controller loop (~every physics timestep)
  IF dt > 0 {
    SET thrott to min(1, max(hover_throttle_level + dthrott,0)).
    SET t0 TO TIME:SECONDS.
    LOG logline TO logfile.
  }

  // print out at 2Hz
  IF TIME:SECONDS - t1 > 0.5 {
    PRINT "Elapsed Time: " + ROUND(TIME:SECONDS-start_time, 2) + "s     " AT (0,9).
    PRINT "Vertical Speed: " + ROUND(SHIP:VERTICALSPEED, 5) + "m/s     " AT (0,10).
    PRINT "Throttle: " + ROUND(thrott, 5) + "%     " AT (0,11).
    PRINT "dThrottle: " + ROUND(dthrott, 5) + "     "  AT (0,12).
    SET t1 TO TIME:SECONDS.
  }
  WAIT 0.001. // physics timestep
}

// Burn up for a few seconds to get some space for the parachute!
SET thrott TO 1.
WAIT UNTIL thrott = 1.
WAIT 3.

// Turn off throttle and wait for it.
SET thrott TO 0.
WAIT UNTIL thrott = 0.
curr_engine:SHUTDOWN().

// Open chutes
WAIT UNTIL STAGE:READY.
PRINT "OPENING CHUTES".
STAGE.
WAIT 5.