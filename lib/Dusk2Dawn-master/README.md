# Dusk2Dawn

Minimal Arduino library for sunrise and sunset time.

Given some basic data, such as geographic coordinates and a date, an estimate time of [*apparent* sunrise or sunset](https://www.esrl.noaa.gov/gmd/grad/solcalc/glossary.html#apparentsunrise) is returned in **minutes elapsed since midnight**.

**WARNING**: This is an unauthorized port of [NOAA's Solar Calculator](https://www.esrl.noaa.gov/gmd/grad/solcalc/),
hence the package will remain unlicensed. *Use at your own risk!*

## Installation
### From the Library Manager
1. Launch the Arduino IDE and navigate to *Sketch → Include Library → Manage Libraries*.
2. In the library manager, scroll to *Dusk2Dawn* or enter the name into the search field.
3. Click on the library, then click on the Install button.

### From the ZIP file
1. Download the [ZIP file](https://github.com/dmkishi/Dusk2Dawn/archive/master.zip).
2. Launch the Arduino IDE and navigate to *Sketch → Include Library → Add .ZIP Library...*. From the prompt, select the ZIP just downloaded.

## Usage
```C++
  /*  Multiple instances can be created. Arguments are longitude, latitude, and
   *  time zone offset in hours from UTC.
   *
   *  The first two must be in decimal degrees (DD), e.g. 10.001, versus the
   *  more common degrees, minutes, and seconds format (DMS), e.g. 10° 00′ 3.6″.
   *  The time zone offset can be expressed in float in the few cases where the
   *  the zones are offset by 30 or 45 minutes, e.g. "5.75" for Nepal Standard
   *  Time.
   *
   *  HINT: An easy way to find the longitude and latitude for any location is
   *  to find the spot in Google Maps, right click the place on the map, and
   *  select "What's here?". At the bottom, you’ll see a card with the
   *  coordinates.
   */
  Dusk2Dawn losAngeles(34.0522, -118.2437, -8);
  Dusk2Dawn antarctica(-77.85, 166.6667, 12);


  /*  Available methods are sunrise() and sunset(). Arguments are year, month,
   *  day, and if Daylight Saving Time is in effect.
   */
  int laSunrise  = losAngeles.sunrise(2017, 12, 31, false);
  int laSunset   = losAngeles.sunset(2017, 12, 31, false);
  int antSunrise = antarctica.sunrise(2017, 6, 30, true);


  /*  Time is returned in minutes elapsed since midnight. If no sunrises or
   *  sunsets are expected, a "-1" is returned.
   */
  Serial.println(laSunrise);  // 418
  Serial.println(laSunset);   // 1004
  Serial.println(antSunrise); // -1


  /*  A static method converts the returned time to a 24-hour clock format.
   *  Arguments are a character array and time in minutes.
   */
  char time[6];
  Dusk2Dawn::min2str(time, laSunrise);
  Serial.println(time); // 06:58


 /*  Alternatively, the array could be initialized with a dummy. This may be
  *  easier to remember.
  */
  char time2[] = "00:00";
  Dusk2Dawn::min2str(time2, laSunset);
  Serial.println(time2); // 16:53


  /*  Do some calculations with the minutes, then convert to time.
   */
  int laSolarNoon = laSunrise + (laSunset - laSunrise) / 2;
  char time3[] = "00:00";
  Dusk2Dawn::min2str(time3, laSolarNoon);
  Serial.println(time3); // 11:56


  /*  In case of an error, an error message is given. The static method also
   *  returns a false boolean value for error handling purposes.
   */
  char time4[] = "00:00";
  bool response = Dusk2Dawn::min2str(time4, antSunrise);
  if (response == false) {
    Serial.println(time4); // "ERROR"
    Serial.println("Uh oh!");
  }
```

## History
- **2019-6-18**: Add license warning.
- **2017-2-9**: Bug fix.
- **2017-2-3**: Released.
