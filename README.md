zamulticompx2
=============

ZaMultiCompX2 - LV2 stereo multiband compressor plugin

This is the stereo version of my multiband compressor plugin.
Feel free to leave comments on my blog as all feedback is appreciated.

http://www.zamaudio.com/?p=870

Install instructions:

lv2-dev and lv2-c++-tools are required to compile this LV2 plugin

	git checkout mono
	make && sudo make install
	make clean
	git checkout stereo
	make && sudo make install
