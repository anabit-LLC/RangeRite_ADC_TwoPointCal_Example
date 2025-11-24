# RangeRite_ADC_TwoPointCal_Example
This example Arduino sketch was developed to work with Anabit's open source RangeRite ADC reference design and 
goes with Anabit's tutorial video on using two point calibration method on ADC circuits. Refer to the link below 
to access the video on Anabit's YouTube Channel. 
The RangeRite ADC gets its name from the fact that it supports 9 different voltage ranges: 5 bipolar and 4 unipolar
all generated from a single input power source between 6V and 18V. The RangeRite ADC comes in two resolution 
versions 16 and 18 bit versions. It also comes in two sample rate versions: 100kSPS and 500kSPS. This example
sketch shows you how to set the RangeRite's input votlage range and make a single ADC measurement every 2 sec.
Be sure to look at the initial settings for this sketch including SPI chip select pin, RangeRite resolution
(16 or 18), voltage range, and if you want to use the reset pin. This sketch also allows you to average multiple
readings together and allows you to enter data for a two point calibration to greatly improve the ADC's accuracy.

RangeRite ADC Dev Board Link: https://anabit.co/products/rangerite-18-bit-adc

Two point calibration tutorial video link: 

This example sketch demonstrates how to set the input voltage range and make a single ADC reading
From Texas Instruments ADS868x 16 bit ADC IC family or the ADS869x 18 bit ADC IC family.

Please report any issue with the sketch to the Anabit forum: https://anabit.co/community/forum/analog-to-digital-converters-adcs

Example code developed by Your Anabit LLC © 2025
Licensed under the Apache License, Version 2.0.
