```
&pinnacle {
    status = "okay";
    compatible = "cirque,pinnacle";
    spi-max-frequency = <1000000>;
    dr-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;
    sensitivity = "2x";
    
    // Enable and configure sigmoid acceleration
    sigmoid-acceleration;
    acceleration-factor = <2.5>;
    acceleration-threshold = <4.0>;
    
    // Enable and configure circular scrolling
    circular-scroll;
    circular-scroll-radius = <18>;
    circular-scroll-deadzone = <3>;
    circular-scroll-deadzone-radius = <15>;
    // circular-scroll-direction = "inverted";  // Clockwise = scroll up, counter-clockwise = scroll down
    circular-scroll-key-mode;
    circular-scroll-key-clockwise = <KEY_VOLUMEUP>;
    circular-scroll-key-counterclockwise = <KEY_VOLUMEDOWN>;

};
```