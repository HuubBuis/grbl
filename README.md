New features in the current [CNCL](https://github.com/MetalWorkerTools/CNCL/wiki) release requires some GRBL compiler options changes. These compiler options are implemented in this GRBL fork to make it easy to compile and flash GRBL to an Arduino for running on a lathe. Read the Wiki [**Compiler Options**](https://github.com/MetalWorkerTools/grbl-L/wiki/Changed-Compiler-options) for an explanation of the compiler options.  
I have added spindle sync threading G33 based on [fschill grbl version](https://github.com/fschill/grbl-Mega/tree/spindle_sync) to this Arduino Uno version [GRBL-L](https://github.com/MetalWorkerTools/grbl-L) and the Arduino Mega version [GRBL-L-Mega](https://github.com/MetalWorkerTools/grbl-L-Mega). Read the [threading setup Wiki](https://github.com/MetalWorkerTools/grbl-L-Mega/wiki/Threading-setup-and-use) for an explanation of the threading setup.  
For all other information read the [GRBL Wiki](https://github.com/gnea/grbl/wiki).  
This release barely fits in the Arduino Uno memory. To upload to an Arduino Uno, use AVR board version 1.8.3 and IDE version 1.8.16
