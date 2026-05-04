// =============================================================
//  Wall-Following + Ball Collector -- Arduino
//
//  Default behaviour : wall following (left wall, IR sensors).
//  Override          : when RPi sends 2-byte ball packets, the
//                      robot suspends wall-following and runs the
//                      full TRACK -> COLLECT -> COLLECTED cycle,
//                      then resumes wall-following automatically.
//
//  Arc roller assist:
//    While executing WL_ARC, if a ball is visible the rollers are
//    switched on so the ball can be ingested passively as the robot
//    sweeps past.  The rollers turn off as soon as the break beam
//    fires (ball secured) or the arc ends.  The arc motion itself
//    is never interrupted.
//
//  Post-collection reverse:
//    After collecting a ball, if the front IR reads within the
//    danger band [FRONT_REV_BAND_LO, FRONT_REV_BAND_HI] the robot
//    reverses.  Two cases are handled:
//
//    Case A -- reading rises during reverse (robot was in normal
//    range but just slightly too close):
//      Stop once the reading has increased FRONT_REV_RISE_CLEAR cm
//      above the value recorded at the start of reverse.
//
//    Case B -- reading falls during reverse (robot is in the IR
//    curl-back zone; the sensor wraps and reads high even though
//    the wall is very close):
//      The reading will dip through a minimum then start climbing.
//      Keep reversing until the reading climbs back up to
//      FRONT_STOP_DIST, at which point the sensor is back in the
//      reliable region and the robot is at a safe distance.
//
//  The two cases are distinguished automatically: if the first
//  directional movement of the reading (after a small settling
