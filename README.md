# StairLight

  This is an Arduino sketch relating to automatic lighting for a staircase with the following characteristics:
  
  * An (analogue) motion sensor signal indicates the presence of a body
  * Pressure pads at the top and bottom of the staircase determine the stairs are being climbed
  * An (analogue) light sensor indicates the ambient light level
  * A PWM-controlled LED light illuminates the staircase

  In the presence of a body, the light will be partially illuminated, fading up to the required level
  and fading down when the presence is removed.

  When/if the stairs are climbed, the lights will be fully illuminated, fading up to the required level
  and fading down after a fixed period.

  The operation described can be suppressed while the ambient light exceeds a certain threshold.
  
 More information about the hardware can be found (here)[https://nofirmconclusion.blogspot.com/2021/05/automatic-led-staircase-lighting.html].
