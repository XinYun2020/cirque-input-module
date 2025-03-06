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
    circular-scroll-direction = "inverted";  // Clockwise = scroll up, counter-clockwise = scroll down

};
```

- Circular Scrolling
    Activates when a button is held down and the finger moves in a circular pattern
    Detects the direction of rotation (clockwise or counterclockwise)
    Determines whether to scroll horizontally or vertically based on the initial movement direction
    Translates the circular motion into appropriate scroll events

- sigmoid-acceleration
    - Tracks finger position relative to the starting point
    - Calculates the angle of the finger position
    - Compares consecutive angles to determine rotation direction
    - Applies a deadzone to prevent accidental scrolling from small movements
    - Converts the angular change into scroll events