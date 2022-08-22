Welcome to my repository for my CS-350 class on Embedded Systems and Technologies!

The final project in this class was to build code that worked with the CC3220S board from TI SimpleLink, a microcontroller board, in which while the code was running the board would read back out over the IDE the temperature it was set to and how long ago the temperature was changed. Basically the board was going to act as a thermostat that was sensing the room temperature and the buttons served as inputs to lower or increase the set temperature with a binary variable showcasing the activation or shut down of the heat function.

There were a lot of aspects that needed to be covered considering my level of expertise with C++ however the instructions were clear and reading the DOCUMENTATION proved essential in order to better understand the variables that I was working with.

I believe that some more books covering microcontrollers from the different ones that I have seen within Amazon are definitely in order so that I can be more efficient at this.

Doing research was very important for this project and I am going to utilize that as well as trying to manage my time better.

I tried to make sure that I put each different function off on it's own and I tried to make the code readable as though if I wasn't the one who worked on it and needed to either add a new function or change some outdated code.

About
repository for my CS-350 class, Embedded Systems and Technology.

## Example Summary

Application that toggles an LED(s) using a GPIO pin interrupt.

## Peripherals & Pin Assignments

When this project is built, the SysConfig tool will generate the TI-Driver
configurations into the __ti_drivers_config.c__ and __ti_drivers_config.h__
files. Information on pins and resources used is present in both generated
files. Additionally, the System Configuration file (\*.syscfg) present in the
project may be opened with SysConfig's graphical user interface to determine
pins and resources used.

* `CONFIG_GPIO_LED_0` - Indicates that the board was initialized within
`mainThread()` also toggled by `CONFIG_GPIO_BUTTON_0`
* `CONFIG_GPIO_LED_1` - Toggled by `CONFIG_GPIO_BUTTON_1`
* `CONFIG_GPIO_BUTTON_0` - Toggles `CONFIG_GPIO_LED_0`
* `CONFIG_GPIO_BUTTON_1` - Toggles `CONFIG_GPIO_LED_1`

## BoosterPacks, Board Resources & Jumper Settings

For board specific jumper settings, resources and BoosterPack modifications,
refer to the __Board.html__ file.

> If you're using an IDE such as Code Composer Studio (CCS) or IAR, please
refer to Board.html in your project directory for resources used and
board-specific jumper settings.

The Board.html can also be found in your SDK installation:

        <SDK_INSTALL_DIR>/source/ti/boards/<BOARD>

## Example Usage

* Run the example. `CONFIG_GPIO_LED_0` turns ON to indicate driver
initialization is complete.

* `CONFIG_GPIO_LED_0` is toggled by pushing `CONFIG_GPIO_BUTTON_0`.
* `CONFIG_GPIO_LED_1` is toggled by pushing `CONFIG_GPIO_BUTTON_1`.

## Application Design Details

* The `gpioButtonFxn0`/`gpioButtonFxn1` functions are configured in the driver configuration
file. These functions are called in the context of the GPIO interrupt.

* Not all boards have more than one button, so `CONFIG_GPIO_LED_1` may not be
toggled.

* There is no button de-bounce logic in the example.

TI-RTOS:

* When building in Code Composer Studio, the configuration project will be
imported along with the example. These projects can be found under
\<SDK_INSTALL_DIR>\/kernel/tirtos/builds/\<BOARD\>/(release|debug)/(ccs|gcc).
The configuration project is referenced by the example, so it
will be built first. The "release" configuration has many debug features
disabled. These features include assert checking, logging and runtime stack
checks. For a detailed difference between the "release" and "debug"
configurations, please refer to the TI-RTOS Kernel User's Guide.

FreeRTOS:

* Please view the `FreeRTOSConfig.h` header file for example configuration
information.
