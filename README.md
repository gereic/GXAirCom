# GxAirCom

GxAircom aims to be a complete and open source implementation of the [FANET+ (Fanet + Flarm) protocol](https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt) running on readily available cheap lora modules and interfacing with mobile phones via Bluetooth. It can also act as a Fanet ground station and broadcast recieved FANET information to [OGN](http://wiki.glidernet.org).

For information and documentation see:

- [The PDF quick guide](doc/20200908%20-%20GXAirCom%20-%20Quick%20Guide.pdf) and the [The PDF Documentation](doc/20200723%20-%20GXAirCom%20-%20A%20LoRa%20communication%20device%20for%20free%20flying.pdf)
- Further information can be found in [the wiki](https://github.com/gereic/GXAirCom/wiki) and [the docs folder](doc/). E.g.:
    - [The list of supported hardware](https://github.com/gereic/GXAirCom/wiki/Hardware-supported)
    - [The list of supported smartphone software](https://github.com/gereic/GXAirCom/wiki/Software)
    - [How to update the firmware](https://github.com/gereic/GXAirCom/wiki/Upgrading---updating-the-firmware-using-the-internal-web-interface-and-a-cellphone)
    - [See the video tutorials](https://github.com/gereic/GXAirCom/wiki/Video-Tutorials)

Similar/ related projects are:

- The [SoftRF](https://github.com/lyusupov/SoftRF) project, which has wider hardware and protocol support, but implements only the subset of the FANET protocol that broadcasts and receives locations. It also cannot broadcast FANET and FLARM at the same time.
- The [Skytraxx FANET Source](https://github.com/3s1d/fanet-stm32). The original reference implementation of the FANET standard and upstream of the standard protocol specification and documentation.

[![Donate](https://raw.githubusercontent.com/stefan-niedermann/paypal-donate-button/master/paypal-donate-button.png)](https://www.paypal.com/donate/?business=JD2NRG9RAS8M6&no_recurring=0&item_name=GXAircom&currency_code=EUR)
