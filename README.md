# CubeCell Board Library (W/ FreqChange)
The default "cubecell_board" library does not support dynamic LoRaWAN frequency changes. The region and frequency is set at compile time, and cannot be changed programatically inside of the code uploaded to the CubeCell HTCC-AB01 board.

This repository has taken the default libraries and made changes so that the LoRa radio region/frequency can be changed on the fly by the code uploaded to the CubeCell HTCC-AB01 board. The default library was changed for the CubeCell-LoRa-Weather-Transceiver-FreqChange project, whose repository is located at https://github.com/wodeeken/CubeCell-LoRa-Weather-Transceiver-FreqChange.

# Usage - Platform IO for VSCode on Ubuntu
1. Inside of a project that is already configured for the HelTec HTCC-AB01 board, located the following directory {User's Home}/.platformio/packages/framework-arduinoasrmicro/libraries/
2. Replace the entire directory with the contents of this repository.
3. Changes  
    a. 	This library overloads the LoRaWAN.join method with callback functions keeping the callee aware of the number of attempted joins, successful joins, as well defining the length of the time program attempts to perform a network join. Example: LoRaWAN.join(&incrementJoinCount, &joinSuccessful, joinTimeMS);  
    b. The method LoRaWAN.init now changes the frequency/region to the passed parameter loraWanRegion instead of what is defined by the pre-processor directive ACTIVE_REGION.  

