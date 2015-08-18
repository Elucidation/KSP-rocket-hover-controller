SAS ON.
SET thrott TO 1.
SET dthrott TO 0.


LOCK THROTTLE TO thrott.
LOCK STEERING TO R(0,0,-90) + HEADING(90,90).

PRINT "LAUNCH".
STAGE.

SET g TO KERBIN:MU / KERBIN:RADIUS^2.
LOCK accvec TO SHIP:SENSORS:ACC - SHIP:SENSORS:GRAV.
print ship:sensors:acc + " " + ship:sensors:grav.

set gforce to 0.

// Burn til 100m off ground
WHEN SHIP:ALTITUDE > 100 THEN {
  LOCK gforce TO accvec:MAG / g.
  LOCK dthrott TO 0.05 * (1.0 - gforce).
}

UNTIL SHIP:LIQUIDFUEL < 0.1 {
  PRINT gforce + " " + dthrott.
  SET thrott to thrott + dthrott.
  WAIT 0.1.
}

// Open chutes
STAGE. 