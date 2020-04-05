# NixiePop_VFD

This is a fork of NixiePop by impressivemachines. It is meant for personal use only. This repo is not maintained.

My father had purchased a NixiePop and wanted to use it to display spindle speed on a CNC mill. I modified the C source to eliminate serial communications and automatically scale a voltage signal from the VFD to a rotation rate in RPM. 

In the extremely unlikely case someone has this same use scenario, the signal from the VFD is measured at ADC6. 5 V is scaled to 8000 RPM (8000 on the display). 

While I do not intend on maintaining this repo in any way, if you have questions about how you can make similar modifications yourself, I am happy to help.

I have added over-the-top documentation to the C source to help those who are not familiar with AVR. 
