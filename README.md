# STM32F107 Bootloader - firmware loader from sd-card

stm32cube based firmware bootloader for mkstft28 originally, should be easy to port - initial files from stmcube maker 

## Getting Started
 load into platformio and build - write compiled .vio/build/mkstft28.bin  to 0x8000000 or 0x0 (both same) or use ctrl+alt+u to upload via debugger automagicly 
 
 
### Deployment

Most configuration is in boot_conf.h 
uses spi1 and uart1 by default hard coded into main.c/spisd_diskio.c etc 



## With Thanks to

https://github.com/delwinbest  
https://github.com/robotsrulz/ - source for spisd_diskio patches
?? Roman Stepanov - original source for bootloader ??
## License

A lot of copy pasta code here - various licenses apply 

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
