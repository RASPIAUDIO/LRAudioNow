# True Wireless Stereo with ESP32

This repository contains instructions and resources for setting up a True Wireless Stereo (TWS) system using two ESP32 modules and a Proto board.

## Materials Needed

- 2 ESP32 Modules Proto board ([Muse Proto](https://raspiaudio.com/produit/muse-proto))

## Instructions

1. Connect to the left ESP32 module using a standard Bluetooth connection.
2. Configure the left ESP32 to transmit the right channel data to the right ESP32 using the ESP-NOW protocol. ESP-NOW is preferred for its very low latency.
3. After a predetermined duration, synchronize the left and right ESP32 modules to play their respective audio samples simultaneously.
4. Continue the process of transmitting data and playing audio samples synchronously until the next sample.

## References

- [Tutorial on Raspiaudio Forum](https://forum.raspiaudio.com/t/airpods-like-true-wireless-speaker-bluetooth-experiment-on-esp32-proto/848)
- [GitHub Repository for LRAudioNow](https://github.com/RASPIAUDIO/LRAudioNow)
- [Muse Proto board](https://raspiaudio.com/produit/muse-proto)

Feel free to contribute to this project by submitting issues or pull requests.
